#!/usr/bin/env python3

"""
ROS2 odom localizer node

This node is responsible for localizing the robot in the odom frame.

- It fuses IMU and LiDAR (via scan-to-scan ICP) with an Extended Kalman Filter (EKF).
- IMU is used as a high-rate prediction source (dead reckoning).
- ICP (scan-to-scan, LiDAR odometry) provides slower but drift-reduced relative motion.
- We accumulate scan-to-scan ICP results into a LiDAR odometry pose and use that
  as a pose measurement for the EKF.

This node publishes:

- tf: odom -> base   (continuous local odometry)
- (optional) /odom_local (nav_msgs/Odometry) for debugging

The "global_localizer" node will provide map -> odom and /go1_pose (map frame).
Combining map->odom and odom->base gives the global pose map->base.

Original spec:
    odom_localizer: tf from odom to base_link frame (here we use child "base")
    global_localizer: tf from map to odom frame
    combined: tf from map to base_link frame
"""

import math
from collections import deque
from typing import Deque, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import Header
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import TransformStamped

import tf_transformations
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

# UPDATED: import scan_to_pcd, icp_2d for scan-to-scan ICP
from utils import pose_to_matrix, transform_to_matrix, scan_to_pcd, icp_2d  # type: ignore

class OdomLocalizerNode(Node):
    """
    EKF-based odom localizer:
      - State x = [x, y, yaw, v, yaw_rate, bax, bay, bwz]^T  in odom frame
      - Prediction: IMU (/imu_plugin/out)
      - Measurement: LiDAR odometry from scan-to-scan ICP (pose in odom frame)
    """

    def __init__(self):
        super().__init__("odom_localizer")

        self.get_logger().info("Odom localizer node (EKF + IMU + scan-to-scan ICP) initialized")

        # --- Simulation time (/clock) ---
        self.clock_sub = self.create_subscription(
            Clock, "/clock", self.clock_callback, 10
        )
        self.current_time: Time | None = None

        # --- TF buffer/listener for odom, base, laser frames ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- TF broadcaster for odom -> base ---
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # --- Scan subscription (for ICP) ---
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 1
        )

        # --- IMU subscription (prediction source) ---
        self.imu_sub = self.create_subscription(
            Imu, "/imu_plugin/out", self.imu_callback, 100
        )

        # --- EKF state ---
        # x = [x, y, yaw, v, bax]^T
        self.state_dim = 5
        self.x = np.zeros((self.state_dim, 1))
        self.P = np.eye(self.state_dim) * 0.01

        self.initialized = False
        self.last_imu_time: Time | None = None

        # --- noise parameters (TODO: tuning needed) ---
        self.Q_imu_base = np.diag([
            0.01, 0.01, math.radians(1.0) ** 2,  # x, y, yaw
            0.1,         # v, yaw_rate
            1e-4                # bax
        ])
        self.R_icp_base = np.diag([
            0.03**2, 0.03**2, math.radians(1.2)**2
        ])  # base ICP measurement covariance

        # --- timer: periodically publish odom->base TF & /odom_local ---
        self.tf_timer = self.create_timer(0.01, self.publish_odom_tf_and_msg)

        # === Scan-to-scan ICP (LiDAR odometry) state ===
        self.prev_scan_pcd: np.ndarray | None = None
        self.prev_scan_time: Time | None = None

        # LiDAR odometry pose in odom frame (used as measurement to EKF)
        self.lidar_pose_x = 0.0
        self.lidar_pose_y = 0.0
        self.lidar_pose_yaw = 0.0
        self.lidar_initialized = False

        # ICP parameters (you can tune these)
        self.icp_max_iterations = 50
        self.icp_tolerance = 1e-4
        self.icp_distance_threshold = 0.5  # meters

        # Cache base->laser static transform (optional)
        self.has_base_to_laser = False
        self.T_base_to_laser = np.eye(4)
        self.T_laser_to_base = np.eye(4)

    # =========================================================
    # Callbacks
    # =========================================================

    def clock_callback(self, msg: Clock):
        self.current_time = Time.from_msg(msg.clock)

    def imu_callback(self, msg: Imu):
        """
        EKF prediction step using IMU linear acceleration and angular velocity.
        Also accumulates small SE(2) increments (for potential forwarding/debugging).
        """
        t = Time.from_msg(msg.header.stamp)
        if not self.initialized:
            # Initialize state when first IMU arrives
            self.last_imu_time = t
            self.initialized = True
            return

        dt = (t - self.last_imu_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        self.last_imu_time = t

        # Unpack state
        x, y, yaw, v, bax = self.x.flatten()

        # IMU measurements
        ax_meas = msg.linear_acceleration.x
        wz_meas = msg.angular_velocity.z

        # bias correction
        ax = ax_meas - bax
        wz = wz_meas

        # FIXED: Improved motion model
        # Update yaw first
        yaw_mid = yaw + 0.5 * wz * dt  # midpoint yaw for better integration
        yaw_new = yaw + wz * dt
        yaw_new = self.normalize_angle(yaw_new)

        
        # Update velocity (assuming forward acceleration)
        v_new = v + ax * dt

        # FIXED: Use midpoint yaw for position update (more accurate)
        cos_yaw_mid = math.cos(yaw_mid)
        sin_yaw_mid = math.sin(yaw_mid)
        
        # world-frame displacement
        dx_world = v * cos_yaw_mid * dt + 0.5 * ax * cos_yaw_mid * dt * dt
        dy_world = v * sin_yaw_mid * dt + 0.5 * ax * sin_yaw_mid * dt * dt

        x_new = x + dx_world
        y_new = y + dy_world

        x_pred = np.array([[x_new, y_new, yaw_new, v_new, bax]]).T

        # FIXED: Proper Jacobian for covariance propagation
        F = np.eye(self.state_dim)
        
        # Partial derivatives
        # dx/dyaw
        F[0, 2] = -v * sin_yaw_mid * dt - 0.5 * ax * sin_yaw_mid * dt * dt
        # dx/dv
        F[0, 3] = cos_yaw_mid * dt
        F[0, 4] = -0.5 * cos_yaw_mid * dt * dt # dx/dbax
        
        # dy/dyaw
        F[1, 2] = v * cos_yaw_mid * dt + 0.5 * ax * cos_yaw_mid * dt * dt
        # dy/dv
        F[1, 3] = sin_yaw_mid * dt
        # dy/dbax
        F[1, 4] = -0.5 * sin_yaw_mid * dt * dt
        
        # dv/dbax (velocity affected by x-axis bias)
        F[3, 4] = -dt
        
        Q = self.Q_imu_base * dt
        P_pred = F @ self.P @ F.T + Q

        self.x = x_pred
        self.P = P_pred

    def scan_callback(self, scan: LaserScan):
        """
        Scan-to-scan ICP callback:

        1) Convert LaserScan to point cloud in laser frame.
        2) If first scan: initialize LiDAR odometry pose from current EKF state.
        3) For subsequent scans:
            - Run ICP between previous and current scan to get relative motion
              in the laser frame.
            - Optionally convert this motion to the base frame using base->laser TF.
            - Accumulate this relative motion into a LiDAR odometry pose in odom frame.
            - Use this pose as a measurement z = [x_icp, y_icp, yaw_icp] for EKF.
        """
    def scan_callback(self, scan: LaserScan):
        if not self.initialized:
            return
        if self.current_time is None:
            return

        t_scan = Time.from_msg(scan.header.stamp)

        # 1) scan -> point cloud
        try:
            current_pcd_laser = scan_to_pcd(scan)
        except Exception as e:
            self.get_logger().warn(f"[ICP] scan_to_pcd failed: {e}")
            return

        # 2) First scan: Initialize LiDAR odom from current EKF state
        if self.prev_scan_pcd is None:
            self.prev_scan_pcd = current_pcd_laser
            self.prev_scan_time = t_scan
            
            # Initialize independent accumulator with current best guess (EKF state)
            self.lidar_pose_x = float(self.x[0, 0])
            self.lidar_pose_y = float(self.x[1, 0])
            self.lidar_pose_yaw = float(self.x[2, 0])
            self.lidar_initialized = True

            self.update_base_laser_transform()
            return

        # 3) Run scan-to-scan ICP
        # FIX APPLIED: Swapped arguments (Target=prev, Source=curr)
        try:
            T_prev_to_curr_laser, fitness, inliers = icp_2d(
                previous_pcd=self.prev_scan_pcd,   # Target (Previous)
                current_pcd=current_pcd_laser,     # Source (Current)
                max_iterations=self.icp_max_iterations,
                tolerance=self.icp_tolerance,
                distance_threshold=self.icp_distance_threshold,
            )
        except Exception as e:
            self.get_logger().warn(f"[ICP] icp_2d failed: {e}")
            self.prev_scan_pcd = current_pcd_laser
            return

        # Update previous scan reference
        self.prev_scan_pcd = current_pcd_laser
        self.prev_scan_time = t_scan

        # Extract relative motion (in Laser Frame)
        dx_laser = float(T_prev_to_curr_laser[0, 2])
        dy_laser = float(T_prev_to_curr_laser[1, 2])
        yaw_laser = math.atan2(T_prev_to_curr_laser[1, 0], T_prev_to_curr_laser[0, 0])
        yaw_laser = self.normalize_angle(yaw_laser)

        # 4) Convert to Base Frame
        if self.has_base_to_laser:
            # Build 4x4 from 3x3 result
            T_prev_to_curr_laser_4 = np.eye(4)
            T_prev_to_curr_laser_4[0:2, 0:2] = T_prev_to_curr_laser[0:2, 0:2]
            T_prev_to_curr_laser_4[0:2, 3] = T_prev_to_curr_laser[0:2, 2]

            # Transform: Base_delta = T_laser_base * Laser_delta * T_base_laser
            T_prev_to_curr_base_4 = self.T_laser_to_base @ T_prev_to_curr_laser_4 @ self.T_base_to_laser

            dx_body = float(T_prev_to_curr_base_4[0, 3])
            dy_body = float(T_prev_to_curr_base_4[1, 3])
            yaw_body = math.atan2(T_prev_to_curr_base_4[1, 0], T_prev_to_curr_base_4[0, 0])
            yaw_body = self.normalize_angle(yaw_body)
        else:
            dx_body, dy_body, yaw_body = dx_laser, dy_laser, yaw_laser

        # 5) Accumulate into Independent LiDAR Odometry
        # CRITICAL: Rotate body delta by the PREVIOUS LIDAR YAW, not EKF yaw
        c = math.cos(self.lidar_pose_yaw)
        s = math.sin(self.lidar_pose_yaw)
        
        dx_world = c * dx_body - s * dy_body
        dy_world = s * dx_body + c * dy_body

        self.lidar_pose_x += dx_world
        self.lidar_pose_y += dy_world
        self.lidar_pose_yaw = self.normalize_angle(self.lidar_pose_yaw + yaw_body)

        # 6) EKF update: Use the independent LiDAR pose as measurement
        self.ekf_update_icp(
            self.lidar_pose_x, 
            self.lidar_pose_y, 
            self.lidar_pose_yaw,
            fitness, inliers
        )
        

    # =========================================================
    # EKF / ICP Helper
    # =========================================================

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        return (angle + np.pi) % (2.0 * np.pi) - np.pi

    def ekf_update_icp(self, x_meas: float, y_meas: float, yaw_meas: float,
                    fitness: float, inliers: int):
        """
        EKF measurement update with ICP pose (in odom frame).
        Measurement: z = [x, y, yaw]^T
        """
        # Build measurement noise from ICP quality
        R = self.compute_R_from_icp_quality(self.R_icp_base, fitness, inliers)
        if R is None:
            # Too poor quality, skip update
            return
        # self.get_logger().info(f"Q: {self.Q_imu_base}, R: {R}")
        # Measurement vector (3x1)
        z = np.array([[x_meas],
                    [y_meas],
                    [yaw_meas]])

        # Measurement matrix H: picks [x, y, yaw] from the state
        H = np.zeros((3, self.state_dim))
        H[0, 0] = 1.0  # x
        H[1, 1] = 1.0  # y
        H[2, 2] = 1.0  # yaw

        # Predicted measurement (3x1)
        hx = H @ self.x

        # Innovation
        y = z - hx
        # Normalize yaw innovation
        y[2, 0] = self.normalize_angle(y[2, 0])

        # Innovation covariance
        S = H @ self.P @ H.T + R

        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            self.get_logger().warn("[EKF] S inverse failed, skip update.")
            return

        # Kalman gain
        K = self.P @ H.T @ S_inv  # (n x 3)

        # State update
        x_upd = self.x + K @ y
        # Normalize yaw in state as well
        x_upd[2, 0] = self.normalize_angle(x_upd[2, 0])

        # Joseph form covariance update for numerical stability
        I = np.eye(self.state_dim)
        I_KH = I - K @ H
        P_upd = I_KH @ self.P @ I_KH.T + K @ R @ K.T

        self.x = x_upd
        self.P = P_upd

        self.get_logger().debug(
            f"[EKF] Updated and synchronized ICP pose to "
            f"({self.x[0, 0]:.3f}, {self.x[1, 0]:.3f}, {math.degrees(self.x[2, 0]):.1f}°)"
        )


    def compute_R_from_icp_quality(self, R_base: np.ndarray, fitness: float, inliers: int):
        """
        Simple heuristic:
        - if inliers too small, skip update (return None)
        - otherwise scale R with fitness.

        For scan-to-scan ICP, we currently approximate fitness and inliers.
        You can refine this once you modify icp_2d to return error statistics.
        """
        MIN_INLIERS = 30
        if inliers < MIN_INLIERS:
            return None

        scale = 20 / fitness # need tuning
        return R_base * scale

    # =========================================================
    # Base<->Laser transform helper
    # =========================================================

    def update_base_laser_transform(self):
        """
        Try to lookup static transform base -> laser once and cache it.
        If lookup fails, ICP will approximate laser frame as base frame.
        """
        if self.has_base_to_laser:
            return
        try:
            # FIXED: Use Time() without arguments for latest transform
            tf_msg = self.tf_buffer.lookup_transform("base", "laser", Time())
            T = transform_to_matrix(tf_msg.transform)
            self.T_base_to_laser = T
            self.T_laser_to_base = np.linalg.inv(T)
            self.has_base_to_laser = True
            self.get_logger().info("[ICP] Cached base->laser transform.")
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass  # Will retry on next scan

    # =========================================================
    # TF & Odometry publish
    # =========================================================

    def publish_odom_tf_and_msg(self):
        if not self.initialized:
            return
        if self.current_time is None:
            return

        x, y, yaw, _, _ = self.x.flatten()

        # TF: odom -> base
        quat = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.current_time.to_msg()
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base"
        tf_msg.transform.translation.x = float(x)
        tf_msg.transform.translation.y = float(y)
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = float(quat[0])
        tf_msg.transform.rotation.y = float(quat[1])
        tf_msg.transform.rotation.z = float(quat[2])
        tf_msg.transform.rotation.w = float(quat[3])

        self.tf_broadcaster.sendTransform(tf_msg)

    # =========================================================
    # Small helpers
    # =========================================================

    @staticmethod
    def se2_from_matrix(T: np.ndarray) -> Tuple[float, float, float]:
        dx = float(T[0, 3])
        dy = float(T[1, 3])
        yaw = math.atan2(T[1, 0], T[0, 0])
        return dx, dy, yaw


def main(args=None):
    rclpy.init(args=args)
    node = OdomLocalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()