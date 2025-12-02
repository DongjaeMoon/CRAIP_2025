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
    scan_to_pcd,
    icp_2d,
)


class Particle:
    __slots__ = ("x", "y", "yaw", "weight", "log_weight")

    def __init__(self, x=0.0, y=0.0, yaw=0.0, weight=1.0, log_weight=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.weight = weight
        self.log_weight = log_weight


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
            LaserScan, "/scan", self.scan_callback, 1
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
        self.T_map_to_base = None

        # --- Particle Filter state ---
        self.num_particles = 150
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

        self.icp_counter = 0
        self.icp_every_n_updates = 5  # Run ICP every 5th PF update
        self.smoothed_x = None
        self.smoothed_y = None
        self.smoothed_yaw = None
        self.alpha_pos = 0.3  # Smoothing factor for position
        self.alpha_yaw = 0.2  # Smoothing factor for yaw

        self.map_cache = {}  # Dict: (grid_x, grid_y, radius) -> np.ndarray
        self.map_cache_resolution = 2.0  # Cache grid cell size [m]
        self.map_cache_max_size = 100  # Maximum cache entries

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

        if not self.likelihood_field_building and self.likelihood_field is None:
            self.build_likelihood_field()

    def build_likelihood_field(self):
        """
        Build likelihood field in a timer callback (called once after short delay)
        """
        
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
        
        t_scan = Time.from_msg(scan.header.stamp)
        
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
        # best_p = max(self.particles, key=lambda p: p.weight)
        # T_map_to_base = self.matrix_from_pose_2d(best_p.x, best_p.y, best_p.yaw)

        # or use smoothed pose
        pose = self.get_smoothed_pose()
        if pose is None:
            return
        smooth_x, smooth_y, smooth_yaw = pose
        T_map_to_base = self.matrix_from_pose_2d(smooth_x, smooth_y, smooth_yaw)

        # 7) Update map->odom
        self.update_map_to_odom_from_pose_matrix(T_map_to_base, T_odom_to_base)

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

        now = self.get_clock().now()
        try:
            T_odom_from_base = transform_to_matrix(
                self.tf_buffer.lookup_transform("odom", "base", now.to_msg()).transform
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"[PF] Could not get odom<-base: {e}", throttle_duration_sec=2.0)
            return

        T_map_to_base = self.T_map_to_odom @ T_odom_from_base
        self.publish_go1_pose_from_matrix(T_map_to_base, now.to_msg())

    # =========================================================
    # Particle Filter helpers
    # =========================================================

    def initialize_particles_from_initial_pose(self):
        sigma_x = 0.1
        sigma_y = 0.1
        sigma_yaw = math.radians(3.0)

        self.particles = []
        for _ in range(self.num_particles):
            x = self.init_x + np.random.normal(0.0, sigma_x)
            y = self.init_y + np.random.normal(0.0, sigma_y)
            yaw = self.init_yaw + np.random.normal(0.0, sigma_yaw)
            self.particles.append(Particle(x, y, yaw, weight=1.0 / self.num_particles))

    def pf_prediction(self, dx_odom: float, dy_odom: float, dyaw_odom: float):
        """Motion model"""
        sigma_trans = 0.15
        sigma_rot = math.radians(3.0)

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

        log_weights = []
        for p in self.particles:
            log_w = compute_scan_log_likelihood_endpoint_model(p.x, p.y, p.yaw, scan, self.likelihood_field)
            log_weights.append(log_w)
            
        # Log-Sum-Exp Trick for numerical stability
        max_log_w = max(log_weights)
        weights = []
        for lw in log_weights:
            weights.append(math.exp(lw - max_log_w))
            
        # Normalize
        total_w = sum(weights)
        if total_w > 1e-9:
            for i, p in enumerate(self.particles):
                p.weight = weights[i] / total_w
        else:
            # Recovery: If all particles have 0 probability, add random noise
            self.get_logger().warn("Particle Filter Lost! Adding noise.")
            for p in self.particles:
                p.weight = 1.0 / self.num_particles
                p.x += np.random.normal(0, 0.2)
                p.y += np.random.normal(0, 0.2)

        self.icp_counter += 1
        if self.icp_counter >= self.icp_every_n_updates:
            self.refine_top_particles_with_icp(scan, top_k=1)  # Reduce from 5 to 3
            self.icp_counter = 0

    def refine_top_particles_with_icp(self, scan: LaserScan, top_k: int = 5):
        """Apply ICP to refine top-K particles"""
        
        # Sort by weight
        sorted_indices = sorted(
            range(len(self.particles)),
            key=lambda i: self.particles[i].weight,
            reverse=True
        )
        
        scan_pcd = scan_to_pcd(scan)
        if len(scan_pcd) < 20:
            return
        
        refined = 0
        for idx in sorted_indices[:top_k]:
            p = self.particles[idx]
            
            # Extract local map
            map_pcd = self.get_cached_map_region(p.x, p.y, radius=8.0)
            if map_pcd is None or len(map_pcd) < 30:
                continue
            
            # Transform scan to world
            scan_world = self.transform_scan_to_world(scan_pcd, p.x, p.y, p.yaw)
            
            try:
                # ICP alignment
                T_icp, _, _ = icp_2d(
                    previous_pcd=map_pcd,
                    current_pcd=scan_world,
                    max_iterations=25,
                    tolerance=1e-4,
                    distance_threshold=0.25
                )
                
                T_p = self.se2_from_pose(p.x, p.y, p.yaw)

                # Refined pose: X_new = T_icp * X_p
                T_new = T_icp @ T_p
                # Update particle
                x_new, y_new, yaw_new = T_new[:3, 3].flatten()
                dx   = x_new - p.x
                dy   = y_new - p.y
                dyaw = self.normalize_angle(yaw_new - p.yaw)
                
                # Only apply if refinement is small (sanity check)
                if math.sqrt(dx**2 + dy**2) < 0.5 and abs(dyaw) < math.radians(15):
                    p.x += dx
                    p.y += dy
                    p.yaw = self.normalize_angle(p.yaw + dyaw)
                    refined += 1
                    
            except Exception:
                pass
        
        if refined > 0:
            self.get_logger().debug(f"[ICP] Refined {refined}/{top_k} particles")

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
    def se2_from_pose(x: float, y: float, yaw: float) -> np.ndarray:
        """Construct 3x3 SE(2) transform from 2D pose."""
        c = math.cos(yaw)
        s = math.sin(yaw)
        T = np.eye(3)
        T[0, 0] = c;  T[0, 1] = -s
        T[1, 0] = s;  T[1, 1] =  c
        T[0, 2] = x
        T[1, 2] = y
        return T

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
    
    # ICP helpers
    def extract_map_points_around_pose(self, x: float, y: float, radius: float) -> np.ndarray | None:
        """
        Extract occupied cells from map as point cloud around a pose.
        
        Args:
            x, y: Center position in map frame [m]
            radius: Extraction radius [m]
            
        Returns:
            map_pcd: (N, 2) array of occupied cell coordinates, or None
        """
        if self.map_msg is None:
            return None
        
        info = self.map_msg.info
        width = info.width
        height = info.height
        resolution = info.resolution
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        
        # Convert center to map coordinates
        cx = int((x - origin_x) / resolution)
        cy = int((y - origin_y) / resolution)
        
        # Compute radius in cells
        radius_cells = int(radius / resolution)
        
        # Extract region
        x_min = max(0, cx - radius_cells)
        x_max = min(width, cx + radius_cells)
        y_min = max(0, cy - radius_cells)
        y_max = min(height, cy + radius_cells)
        
        # Find occupied cells in region
        data = np.array(self.map_msg.data, dtype=np.int8).reshape((height, width))
        points = []
        
        for my in range(y_min, y_max):
            for mx in range(x_min, x_max):
                if data[my, mx] >= 50:  # Occupied threshold
                    # Convert to world coordinates
                    px = origin_x + mx * resolution
                    py = origin_y + my * resolution
                    points.append([px, py])
        
        if len(points) == 0:
            return None
        
        return np.array(points, dtype=np.float32)

    def transform_scan_to_world(self, scan_pcd: np.ndarray, 
                                x: float, y: float, yaw: float) -> np.ndarray:
        """
        Transform scan point cloud from laser frame to world frame.
        
        Args:
            scan_pcd: (N, 2) scan points in laser frame
            x, y, yaw: Robot pose in world frame
            
        Returns:
            world_pcd: (N, 2) scan points in world frame
        """
        # Rotation matrix
        c = math.cos(yaw)
        s = math.sin(yaw)
        R = np.array([[c, -s], [s, c]])
        
        # Transform: world = R @ scan + translation
        world_pcd = (R @ scan_pcd.T).T + np.array([x, y])
        
        return world_pcd

    def get_smoothed_pose(self):
        """Compute weighted average of particles and smooth over time"""
        # Weighted average of top particles
        sorted_particles = sorted(self.particles, key=lambda p: p.weight, reverse=True)
        top_k = min(10, len(sorted_particles))
        
        total_weight = sum(p.weight for p in sorted_particles[:top_k])
        if total_weight == 0:
            return None
            
        avg_x = sum(p.x * p.weight for p in sorted_particles[:top_k]) / total_weight
        avg_y = sum(p.y * p.weight for p in sorted_particles[:top_k]) / total_weight
        
        # Circular average for yaw
        sin_sum = sum(math.sin(p.yaw) * p.weight for p in sorted_particles[:top_k])
        cos_sum = sum(math.cos(p.yaw) * p.weight for p in sorted_particles[:top_k])
        avg_yaw = math.atan2(sin_sum, cos_sum)
        
        # EMA smoothing
        if self.smoothed_x is None:
            self.smoothed_x = avg_x
            self.smoothed_y = avg_y
            self.smoothed_yaw = avg_yaw
        else:
            self.smoothed_x = self.alpha_pos * avg_x + (1 - self.alpha_pos) * self.smoothed_x
            self.smoothed_y = self.alpha_pos * avg_y + (1 - self.alpha_pos) * self.smoothed_y
            
            # Smooth yaw using circular interpolation
            delta_yaw = self.normalize_angle(avg_yaw - self.smoothed_yaw)
            self.smoothed_yaw = self.normalize_angle(self.smoothed_yaw + self.alpha_yaw * delta_yaw)
        
        return self.smoothed_x, self.smoothed_y, self.smoothed_yaw
    def get_cached_map_region(self, x: float, y: float, radius: float) -> np.ndarray | None:
        """
        Get map point cloud with caching.
        
        Caches extracted map regions on a spatial grid to avoid redundant extractions
        when multiple nearby particles request the same region.
        
        Args:
            x, y: Center position in map frame [m]
            radius: Extraction radius [m]
            
        Returns:
            map_pcd: (N, 2) array of occupied cell coordinates, or None
        """
        # Snap to cache grid
        grid_x = int(round(x / self.map_cache_resolution))
        grid_y = int(round(y / self.map_cache_resolution))
        
        # Round radius to nearest 0.5m to increase cache hits
        cache_radius = round(radius * 2) / 2  # 8.0 -> 8.0, 8.3 -> 8.5
        
        cache_key = (grid_x, grid_y, cache_radius)
        
        # Check cache
        if cache_key in self.map_cache:
            return self.map_cache[cache_key]
        
        # Cache miss - extract from map
        # Use grid center for extraction (not exact x, y)
        center_x = grid_x * self.map_cache_resolution
        center_y = grid_y * self.map_cache_resolution
        
        map_pcd = self.extract_map_points_around_pose(center_x, center_y, cache_radius)
        
        # Store in cache
        self.map_cache[cache_key] = map_pcd
        
        # Enforce cache size limit (LRU-like behavior)
        if len(self.map_cache) > self.map_cache_max_size:
            # Remove oldest entry (first key in dict)
            # In Python 3.7+, dicts maintain insertion order
            oldest_key = next(iter(self.map_cache))
            del self.map_cache[oldest_key]
        
        return map_pcd


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