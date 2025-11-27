#!/usr/bin/env python3

"""
ROS2 global localizer node - FIXED VERSION

Key fix: Non-blocking likelihood field building to prevent node from hanging
"""

import math
from typing import List

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, TransformStamped

import tf_transformations
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from utils import (
    pose_to_matrix,
    transform_to_matrix,
    build_likelihood_field_from_map,
    compute_scan_log_likelihood_endpoint_model,
)


class Particle:
    __slots__ = ("x", "y", "yaw", "weight")

    def __init__(self, x=0.0, y=0.0, yaw=0.0, weight=1.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.weight = weight


class GlobalLocalizerNode(Node):
    def __init__(self):
        super().__init__("global_localizer")
        self.get_logger().info("Global localizer node (PF + map + LiDAR) initialized")

        # --- Simulation time (/clock) ---
        self.clock_sub = self.create_subscription(
            Clock, "/clock", self.clock_callback, 10
        )
        self.current_time: Time | None = None

        # --- TF buffer/listener ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- TF broadcaster (map -> odom) ---
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_pub_interval = 0.1
        self.tf_timer = self.create_timer(self.tf_pub_interval, self.tf_timer_callback)

        # --- Map & Scan subscriptions ---
        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, map_qos
        )

        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )

        self.map_msg: OccupancyGrid | None = None

        # Likelihood field (distance field) built from /map for sensor model
        self.likelihood_field = None
        self.likelihood_field_building = False

        # --- Parameters: initial pose from launch file ---
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 1.0)
        self.declare_parameter("yaw", 0.0)
        self.init_x = float(self.get_parameter("x").value)
        self.init_y = float(self.get_parameter("y").value)
        self.init_yaw = float(self.get_parameter("yaw").value)

        self.z = 0.33  # robot base height
        q_init = tf_transformations.quaternion_from_euler(0.0, 0.0, self.init_yaw)
        self.T_map_to_base_init = pose_to_matrix(
            [self.init_x, self.init_y, self.z, q_init[0], q_init[1], q_init[2], q_init[3]]
        )

        # --- Particle Filter state ---
        self.num_particles = 200
        self.particles: List[Particle] = []
        self.pf_initialized = False

        self.last_scan_time: Time | None = None
        self.last_odom_to_base_T: np.ndarray | None = None

        # --- map->odom (global correction) ---
        self.T_map_to_odom: np.ndarray | None = None

        # --- /go1_pose (map frame) publisher ---
        self.go1_pose_pub = self.create_publisher(PoseStamped, "/go1_pose", 10)

        # Build likelihood field in background after short delay
        self.likelihood_build_timer = None

    # =========================================================
    # Callbacks
    # =========================================================

    def clock_callback(self, msg: Clock):
        self.current_time = Time.from_msg(msg.clock)

    def map_callback(self, msg: OccupancyGrid):
        """
        Map callback - save map and schedule likelihood field building
        """
        self.get_logger().info(f"[PF] Map received: {msg.info.width}x{msg.info.height}, res={msg.info.resolution:.3f}m")
        self.map_msg = msg

        # Schedule likelihood field building with a short delay (non-blocking)
        if self.likelihood_build_timer is None and not self.likelihood_field_building:
            self.likelihood_build_timer = self.create_timer(0.5, self.build_likelihood_field_delayed)

    def build_likelihood_field_delayed(self):
        """
        Build likelihood field in a timer callback (called once after short delay)
        """
        if self.likelihood_field_building:
            return
        
        if self.map_msg is None:
            return

        self.likelihood_field_building = True
        self.get_logger().info("[PF] Building likelihood field (this may take a moment)...")
        
        try:
            self.likelihood_field = build_likelihood_field_from_map(self.map_msg)
            self.get_logger().info("[PF] Likelihood field built successfully!")
        except Exception as e:
            self.get_logger().error(f"[PF] Failed to build likelihood field: {e}")
            import traceback
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")
            self.likelihood_field = None
        finally:
            self.likelihood_field_building = False
            # Cancel the timer so it only runs once
            if self.likelihood_build_timer is not None:
                self.likelihood_build_timer.cancel()
                self.likelihood_build_timer = None

    def scan_callback(self, scan: LaserScan):
        """
        Particle Filter iteration triggered by each LaserScan
        """
        if self.current_time is None:
            return
        if self.map_msg is None:
            return
        
        # Wait for likelihood field to be ready
        if self.likelihood_field is None:
            if self.likelihood_field_building:
                # Still building, skip this scan
                return
            else:
                # Building failed or not started
                self.get_logger().warn("[PF] No likelihood field available", throttle_duration_sec=5.0)
                return

        t_scan_msg = scan.header.stamp
        t_scan = Time.from_msg(t_scan_msg)

        # 1) Get odom->base TF
        try:
            odom_to_base_tf = self.tf_buffer.lookup_transform(
                "odom", "base", t_scan_msg
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"[PF] Could not get odom->base: {e}", throttle_duration_sec=2.0)
            return

        T_odom_to_base = transform_to_matrix(odom_to_base_tf.transform)

        # Initialize PF on first scan
        if not self.pf_initialized:
            self.initialize_particles_from_initial_pose()
            self.last_odom_to_base_T = T_odom_to_base
            self.last_scan_time = t_scan
            self.pf_initialized = True

            # Initial map->odom = T_map_to_base_init * inv(T_odom_to_base)
            self.update_map_to_odom_from_pose_matrix(self.T_map_to_base_init, T_odom_to_base)
            self.publish_go1_pose_from_matrix(self.T_map_to_base_init, t_scan_msg)
            self.get_logger().info("[PF] Particle filter initialized and broadcasting map->odom")
            return

        if self.last_odom_to_base_T is None:
            self.last_odom_to_base_T = T_odom_to_base
            self.last_scan_time = t_scan
            return

        # 2) Compute motion delta
        T_prev = self.last_odom_to_base_T
        T_curr = T_odom_to_base
        T_prev_inv = np.linalg.inv(T_prev)
        T_delta = T_prev_inv @ T_curr

        self.last_odom_to_base_T = T_odom_to_base
        self.last_scan_time = t_scan

        dx_odom, dy_odom, dyaw_odom = self.se2_from_matrix(T_delta)

        # 3) PF prediction
        self.pf_prediction(dx_odom, dy_odom, dyaw_odom)

        # 4) PF sensor update
        self.pf_update_weights(scan)

        # 5) Resample
        self.resample_particles()

        # 6) Best particle
        best_p = max(self.particles, key=lambda p: p.weight)
        T_map_to_base = self.matrix_from_pose_2d(best_p.x, best_p.y, best_p.yaw)

        # 7) Update map->odom
        self.update_map_to_odom_from_pose_matrix(T_map_to_base, T_odom_to_base)

        # 8) Publish /go1_pose
        self.publish_go1_pose_from_matrix(T_map_to_base, t_scan_msg)

    def tf_timer_callback(self):
        """
        Periodically publish map->odom TF
        """
        if self.current_time is None:
            return
        if self.T_map_to_odom is None:
            return

        translation = self.T_map_to_odom[:3, 3]
        quaternion = tf_transformations.quaternion_from_matrix(self.T_map_to_odom)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.current_time.to_msg()
        tf_msg.header.frame_id = "map"
        tf_msg.child_frame_id = "odom"
        tf_msg.transform.translation.x = float(translation[0])
        tf_msg.transform.translation.y = float(translation[1])
        tf_msg.transform.translation.z = float(translation[2])
        tf_msg.transform.rotation.x = float(quaternion[0])
        tf_msg.transform.rotation.y = float(quaternion[1])
        tf_msg.transform.rotation.z = float(quaternion[2])
        tf_msg.transform.rotation.w = float(quaternion[3])
        
        self.tf_broadcaster.sendTransform(tf_msg)

    # =========================================================
    # Particle Filter helpers
    # =========================================================

    def initialize_particles_from_initial_pose(self):
        sigma_x = 0.2
        sigma_y = 0.2
        sigma_yaw = math.radians(10.0)

        self.particles = []
        for _ in range(self.num_particles):
            x = self.init_x + np.random.normal(0.0, sigma_x)
            y = self.init_y + np.random.normal(0.0, sigma_y)
            yaw = self.init_yaw + np.random.normal(0.0, sigma_yaw)
            self.particles.append(Particle(x, y, yaw, weight=1.0 / self.num_particles))

    def pf_prediction(self, dx_odom: float, dy_odom: float, dyaw_odom: float):
        """Motion model"""
        sigma_trans = 0.02
        sigma_rot = math.radians(2.0)

        for p in self.particles:
            noisy_dx = dx_odom + np.random.normal(0.0, sigma_trans)
            noisy_dy = dy_odom + np.random.normal(0.0, sigma_trans)
            noisy_dyaw = dyaw_odom + np.random.normal(0.0, sigma_rot)

            c = math.cos(p.yaw)
            s = math.sin(p.yaw)

            global_dx = c * noisy_dx - s * noisy_dy
            global_dy = s * noisy_dx + c * noisy_dy

            p.x += global_dx
            p.y += global_dy
            p.yaw = self.normalize_angle(p.yaw + noisy_dyaw)

    def pf_update_weights(self, scan: LaserScan):
        """Sensor model"""
        if self.map_msg is None:
            return
        if self.likelihood_field is None:
            return

        N = len(self.particles)
        if N == 0:
            return

        # Compute log-likelihood for each particle
        log_weights = []
        for p in self.particles:
            try:
                log_w = compute_scan_log_likelihood_endpoint_model(
                    particle_x=p.x,
                    particle_y=p.y,
                    particle_yaw=p.yaw,
                    scan=scan,
                    likelihood_field=self.likelihood_field,
                    max_beams=60,
                    sigma_hit=0.2,
                    z_hit=0.9,
                    z_rand=0.1,
                )
                log_weights.append(log_w)
            except Exception as e:
                self.get_logger().warn(f"[PF] Likelihood computation failed: {e}", throttle_duration_sec=5.0)
                log_weights.append(-1e10)  # Very low weight

        # Convert to normalized linear weights
        max_log_w = max(log_weights)
        linear_weights = [math.exp(lw - max_log_w) for lw in log_weights]

        sum_w = sum(linear_weights)
        if sum_w <= 0.0 or not math.isfinite(sum_w):
            uniform_w = 1.0 / N
            for p in self.particles:
                p.weight = uniform_w
            return

        inv_sum_w = 1.0 / sum_w
        for p, w in zip(self.particles, linear_weights):
            p.weight = w * inv_sum_w

    def resample_particles(self):
        """Low-variance resampling"""
        N = len(self.particles)
        if N == 0:
            return

        weights = np.array([p.weight for p in self.particles], dtype=float)
        sum_w = np.sum(weights)
        if sum_w <= 0.0:
            for p in self.particles:
                p.weight = 1.0 / N
            return

        weights /= sum_w

        new_particles: List[Particle] = []
        r = np.random.uniform(0.0, 1.0 / N)
        c = weights[0]
        i = 0
        for m in range(N):
            U = r + m * (1.0 / N)
            while U > c and i < N - 1:
                i += 1
                c += weights[i]
            p = self.particles[i]
            new_particles.append(Particle(p.x, p.y, p.yaw, weight=1.0 / N))
        self.particles = new_particles

    # =========================================================
    # Transform helpers
    # =========================================================

    @staticmethod
    def normalize_angle(a: float) -> float:
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    @staticmethod
    def matrix_from_pose_2d(x: float, y: float, yaw: float) -> np.ndarray:
        """Construct 4x4 transformation matrix from 2D pose"""
        T = np.eye(4)
        c = math.cos(yaw)
        s = math.sin(yaw)
        T[0, 0] = c
        T[0, 1] = -s
        T[1, 0] = s
        T[1, 1] = c
        T[0, 3] = x
        T[1, 3] = y
        T[2, 3] = 0.33  # robot base height
        return T

    @staticmethod
    def se2_from_matrix(T: np.ndarray):
        dx = float(T[0, 3])
        dy = float(T[1, 3])
        yaw = math.atan2(T[1, 0], T[0, 0])
        return dx, dy, yaw

    def update_map_to_odom_from_pose_matrix(self, T_map_to_base: np.ndarray, T_odom_to_base: np.ndarray):
        """
        T_map_to_odom = T_map_to_base * inv(T_odom_to_base)
        """
        T_odom_to_base_inv = np.linalg.inv(T_odom_to_base)
        self.T_map_to_odom = T_map_to_base @ T_odom_to_base_inv

    def publish_go1_pose_from_matrix(self, T_map_to_base: np.ndarray, stamp):
        """Publish pose in map frame"""
        go1 = PoseStamped()
        go1.header.stamp = stamp
        go1.header.frame_id = "map"

        go1.pose.position.x = float(T_map_to_base[0, 3])
        go1.pose.position.y = float(T_map_to_base[1, 3])
        go1.pose.position.z = float(T_map_to_base[2, 3])

        quat = tf_transformations.quaternion_from_matrix(T_map_to_base)
        go1.pose.orientation.x = float(quat[0])
        go1.pose.orientation.y = float(quat[1])
        go1.pose.orientation.z = float(quat[2])
        go1.pose.orientation.w = float(quat[3])

        self.go1_pose_pub.publish(go1)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalLocalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()