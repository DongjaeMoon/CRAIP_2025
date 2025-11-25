#!/usr/bin/env python3

"""
ROS2 odom localizer node

This node is responsible for localizing the robot in the odom frame.

- It fuses IMU and LiDAR (via scan-to-map ICP) with an Extended Kalman Filter (EKF).
- IMU is used as a high-rate prediction source (dead reckoning).
- ICP (scan-to-map) provides slower but more accurate pose measurements.
- Because ICP takes time to compute, this node uses a simple "forwarding" scheme:
  the ICP pose is computed for the scan time (t_scan) and then forwarded to the
  current time using the IMU motion increments between t_scan and now.

This node publishes:

- tf: odom -> base   (continuous local odometry)
- (optional) /odom_local (nav_msgs/Odometry) for debugging

The "global_localizer" node will provide map -> odom and /go1_pose (map frame).
Combining map->odom and odom->base gives the global pose map->base.

Original spec:
    odom_localizer: tf from odom to base_link frame
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

from utils import pose_to_matrix, transform_to_matrix   # 이미 제공되어 있다고 가정


class OdomLocalizerNode(Node):
    """
    EKF-based odom localizer:
      - State x = [x, y, yaw, v, yaw_rate, bax, bay, bwz]^T  in odom frame
      - Prediction: IMU (/imu_plugin/out)
      - Measurement: ICP (scan-to-map) pose in odom frame (with forwarding)
    """

    def __init__(self):
        super().__init__("odom_localizer")

        self.get_logger().info("Odom localizer node (EKF + IMU + ICP) initialized")

        # --- Simulation time (/clock) ---
        self.clock_sub = self.create_subscription(
            Clock, "/clock", self.clock_callback, 10
        )
        self.current_time: Time | None = None

        # --- TF buffer/listener for map, odom, base, laser frames ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- TF broadcaster for odom -> base ---
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # --- Map & Scan subscriptions (for ICP) ---
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

        # --- IMU subscription (prediction source) ---
        self.imu_sub = self.create_subscription(
            Imu, "/imu_plugin/out", self.imu_callback, 100
        )

        # --- Debug Odometry publisher (optional) ---
        self.odom_pub = self.create_publisher(Odometry, "/odom_local", 10)

        # --- EKF state ---
        # x = [x, y, yaw, v, yaw_rate, bax, bay, bwz]^T
        self.state_dim = 8
        self.x = np.zeros((self.state_dim, 1))
        self.P = np.eye(self.state_dim) * 0.01

        self.initialized = False
        self.last_imu_time: Time | None = None

        # --- IMU increment history for ICP forwarding ---
        # Each entry: (stamp: Time, dx, dy, dyaw)
        self.imu_increments: Deque[Tuple[Time, float, float, float]] = deque(maxlen=2000)

        # --- noise parameters (TODO: 튜닝 필요) ---
        self.Q_imu_base = np.diag([
            0.01, 0.01, math.radians(1.0),  # x,y,yaw
            0.1, math.radians(5.0),         # v, yaw_rate
            1e-4, 1e-4, 1e-5                # bax, bay, bwz
        ])
        self.R_icp_base = np.diag([
            0.05**2, 0.05**2, math.radians(2.0)**2
        ])  # ICP 측정 기본 공분산

        # --- timer: 주기적으로 odom->base TF & /odom_local publish ---
        self.tf_timer = self.create_timer(0.01, self.publish_odom_tf_and_msg)

    # =========================================================
    # Callbacks
    # =========================================================

    def clock_callback(self, msg: Clock):
        self.current_time = Time.from_msg(msg.clock)

    def map_callback(self, msg: OccupancyGrid):
        self.map_msg = msg

    def imu_callback(self, msg: Imu):
        """
        EKF prediction step using IMU linear acceleration and angular velocity.
        Also accumulates small SE(2) increments for forwarding.
        """
        t = Time.from_msg(msg.header.stamp)
        if not self.initialized:
            # 처음 IMU가 들어오는 시점에 상태 초기화 (0 근처)
            self.x[:] = 0.0
            self.P = np.eye(self.state_dim) * 0.01
            self.last_imu_time = t
            self.initialized = True
            return

        dt = (t - self.last_imu_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        self.last_imu_time = t

        # Unpack state
        x, y, yaw, v, yaw_rate, bax, bay, bwz = self.x.flatten()

        # IMU measurements
        ax_meas = msg.linear_acceleration.x
        ay_meas = msg.linear_acceleration.y
        wz_meas = msg.angular_velocity.z

        # bias 보정
        ax = ax_meas - bax
        ay = ay_meas - bay
        wz = wz_meas - bwz

        # 상태 예측 (아주 단순한 2D 모델)
        yaw_new = yaw + wz * dt
        v_new = v + ax * dt  # body x축 가속도만 사용한다는 가정

        # world frame 속도
        vx_world = v_new * math.cos(yaw_new)
        vy_world = v_new * math.sin(yaw_new)

        x_new = x + vx_world * dt
        y_new = y + vy_world * dt

        x_pred = np.array([[x_new, y_new, yaw_new, v_new, wz, bax, bay, bwz]]).T

        # TODO: F, Q 계산해서 self.P 업데이트 (여기서는 단순히 Q 더하는 형태로 템플릿)
        F = np.eye(self.state_dim)  # 실제 구현 시 Jacobian 계산 필요
        Q = self.Q_imu_base * dt
        P_pred = F @ self.P @ F.T + Q

        # IMU increment (forwarding용) 저장
        dx = x_new - x
        dy = y_new - y
        dyaw = self.normalize_angle(yaw_new - yaw)
        self.imu_increments.append((t, dx, dy, dyaw))

        self.x = x_pred
        self.P = P_pred

    def scan_callback(self, scan: LaserScan):
        """
        1) scan 시각 t_scan에서의 map->odom, odom->base, base->laser TF 조회
        2) scan-to-map ICP 실행 → map frame에서 base pose (t_scan 시점)
        3) map pose를 odom pose로 변환 (t_scan)
        4) IMU increments 로 t_scan → t_now 동안 forward
        5) EKF update (ICP measurement)
        """
        if not self.initialized:
            return
        if self.map_msg is None:
            return
        if self.current_time is None:
            return

        t_scan_msg = scan.header.stamp
        t_scan = Time.from_msg(t_scan_msg)
        t_now = self.current_time

        # 1) TF lookup at t_scan
        try:
            map_to_odom_tf = self.tf_buffer.lookup_transform(
                "map", "odom", t_scan_msg
            )
            odom_to_base_tf = self.tf_buffer.lookup_transform(
                "odom", "base", t_scan_msg
            )
            base_to_laser_tf = self.tf_buffer.lookup_transform(
                "base", "laser", t_scan_msg
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"[ICP] TF lookup failed: {e}")
            return

        T_map_to_odom = transform_to_matrix(map_to_odom_tf.transform)
        T_odom_to_base = transform_to_matrix(odom_to_base_tf.transform)
        T_base_to_laser = transform_to_matrix(base_to_laser_tf.transform)

        # 2) scan-to-map ICP (초기값: map→base = map→odom ∘ odom→base)
        T_map_to_base_init = T_map_to_odom @ T_odom_to_base

        try:
            T_map_to_base_icp, fitness, inliers, converged = self.run_scan_to_map_icp(
                scan, self.map_msg, T_map_to_base_init, T_base_to_laser
            )
        except Exception as e:
            self.get_logger().warn(f"[ICP] run_scan_to_map_icp error: {e}")
            return

        if not converged:
            return

        # 3) map pose -> odom pose at t_scan: T_odom_to_base_icp_scan
        T_map_to_odom_inv = np.linalg.inv(T_map_to_odom)
        T_odom_to_base_icp_scan = T_map_to_odom_inv @ T_map_to_base_icp

        x_icp_scan, y_icp_scan, yaw_icp_scan = self.se2_from_matrix(T_odom_to_base_icp_scan)

        # 4) forwarding: t_scan → t_now 동안의 IMU Δpose 합산
        dx_imu, dy_imu, dyaw_imu = self.integrate_imu_delta(t_scan, t_now)

        x_icp_now = x_icp_scan + dx_imu
        y_icp_now = y_icp_scan + dy_imu
        yaw_icp_now = self.normalize_angle(yaw_icp_scan + dyaw_imu)

        # 5) EKF update with measurement z = [x_icp_now, y_icp_now, yaw_icp_now]
        self.ekf_update_icp(x_icp_now, y_icp_now, yaw_icp_now, fitness, inliers)

    # =========================================================
    # EKF / ICP Helper
    # =========================================================

    def integrate_imu_delta(self, t_scan: Time, t_now: Time) -> Tuple[float, float, float]:
        dx_sum, dy_sum, dyaw_sum = 0.0, 0.0, 0.0
        for stamp, dx, dy, dyaw in self.imu_increments:
            if stamp <= t_scan:
                continue
            if stamp > t_now:
                break
            dx_sum += dx
            dy_sum += dy
            dyaw_sum += dyaw
        return dx_sum, dy_sum, dyaw_sum

    def ekf_update_icp(self, x_meas: float, y_meas: float, yaw_meas: float,
                       fitness: float, inliers: int):
        """
        EKF measurement update with ICP pose (in odom frame).
        z = [x, y, yaw]^T
        H = [ I_3x3  0_3x5 ]
        R = scaled by ICP quality (fitness, inliers)
        """
        # quality-based R (간단 템플릿)
        R = self.compute_R_from_icp_quality(self.R_icp_base, fitness, inliers)
        if R is None:
            return

        # H: 3x8
        H = np.zeros((3, self.state_dim))
        H[0, 0] = 1.0  # x
        H[1, 1] = 1.0  # y
        H[2, 2] = 1.0  # yaw

        z = np.array([[x_meas, y_meas, yaw_meas]]).T
        hx = np.array([[self.x[0, 0], self.x[1, 0], self.x[2, 0]]]).T

        y = z - hx
        y[2, 0] = self.normalize_angle(y[2, 0])

        S = H @ self.P @ H.T + R
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            self.get_logger().warn("[EKF] S inverse failed, skip update.")
            return

        K = self.P @ H.T @ S_inv
        x_upd = self.x + K @ y
        P_upd = (np.eye(self.state_dim) - K @ H) @ self.P

        x_upd[2, 0] = self.normalize_angle(x_upd[2, 0])

        self.x = x_upd
        self.P = P_upd

    def compute_R_from_icp_quality(self, R_base: np.ndarray, fitness: float, inliers: int):
        """
        Simple heuristic:
        - if inliers too small, skip update (return None)
        - otherwise scale R with fitness
        """
        MIN_INLIERS = 50
        if inliers < MIN_INLIERS:
            return None

        alpha = 2.0  # tuning
        scale = 1.0 + alpha * fitness
        return R_base * scale

    # =========================================================
    # ICP 관련 (템플릿)
    # =========================================================

    def run_scan_to_map_icp(self,
                            scan: LaserScan,
                            map_msg: OccupancyGrid,
                            T_map_to_base_init: np.ndarray,
                            T_base_to_laser: np.ndarray
                            ) -> Tuple[np.ndarray, float, int, bool]:
        """
        Scan-to-map ICP (2D):

        - scan: LaserScan in 'laser' frame
        - map_msg: OccupancyGrid in 'map' frame
        - T_map_to_base_init: initial guess of base pose in map frame (4x4)
        - T_base_to_laser: static transform from base to laser (4x4)

        Returns:
            T_map_to_base_icp (4x4), fitness, inliers, converged
        """
        # TODO:
        #   1) LaserScan -> point cloud in laser frame
        #   2) Transform to map frame using T_map_to_base_init @ T_base_to_laser
        #   3) Prepare map representation (distance field / occupied points)
        #   4) Run ICP iteration (point-to-map)
        #
        # 여기서는 템플릿으로 "초기값 그대로"와 가짜 fitness/inliers만 반환.
        T_map_to_base_icp = T_map_to_base_init.copy()
        fitness = 0.1
        inliers = 200
        converged = True
        return T_map_to_base_icp, fitness, inliers, converged

    # =========================================================
    # TF & Odometry publish
    # =========================================================

    def publish_odom_tf_and_msg(self):
        if not self.initialized:
            return
        if self.current_time is None:
            return

        x, y, yaw, v, yaw_rate, bax, bay, bwz = self.x.flatten()

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

        # Debug odometry
        odom = Odometry()
        odom.header = tf_msg.header
        odom.child_frame_id = "base"
        odom.pose.pose.position.x = float(x)
        odom.pose.pose.position.y = float(y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = tf_msg.transform.rotation
        odom.twist.twist.linear.x = float(v)
        odom.twist.twist.angular.z = float(yaw_rate)
        # TODO: odom.pose.covariance 에 self.P 일부 복사
        self.odom_pub.publish(odom)

    # =========================================================
    # Small helpers
    # =========================================================

    @staticmethod
    def normalize_angle(a: float) -> float:
        return (a + math.pi) % (2.0 * math.pi) - math.pi

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
