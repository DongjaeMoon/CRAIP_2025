#!/usr/bin/env python3
import math
import time
import threading
from typing import Optional, Dict, List

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Float32MultiArray

# =========================
# FIXED GOAL (map frame)
# =========================
GOAL_X = 0.0
GOAL_Y = 12.0
STOP_BEFORE_GOAL = 0.3

# =========================
# BOX CANDIDATES
# =========================
BOX_CANDIDATES: List[Dict[str, float]] = [
    {"name": "cand2", "box_x": 2.0, "box_y": 12.0},
    {"name": "cand1", "box_x": 0.0, "box_y": 14.0},
    {"name": "cand3", "box_x": -2.0, "box_y": 12.0},
]

# =========================
# Perception
# =========================
TOPIC_LABELS = "/detections/labels"
TOPIC_DISTS  = "/detections/distances"
BOX_LABEL = "box"
DIST_MAX = 3.0

# =========================
# Sampling
# =========================
SAMPLES = 10
SAMPLE_INTERVAL = 0.08
MIN_HITS = 2

# =========================
# Navigation Constants
# =========================
STEP_TIMEOUT = 40.0
SETTLE_TIME = 0.7

# [수정] 도달 판정 기준 강화
GOAL_RADIUS = 0.3       # 1.0m는 너무 커서 0.3m로 줄임 (상황에 맞게 조정)
GOAL_YAW_TOLERANCE = 0.15  # 약 8.5도 오차 허용 (rad)

# =========================
# Geometry
# =========================
OBS_OFFSET = 1.9
PRE_OFFSET = 0.75

TOPIC_LOCAL_POSE = "/go1_pose"


def yaw_to_quaternion(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

# [추가] 쿼터니언 -> Yaw 변환 함수
def quaternion_to_yaw(q) -> float:
    # q는 geometry_msgs.msg.Quaternion 또는 유사 객체 (x, y, z, w)
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

# [추가] 각도 정규화 (-PI ~ PI)
def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class Mission4Box(Node):
    def __init__(self):
        super().__init__("mission4_box")

        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)

        self.pose_sub = self.create_subscription(PoseStamped, TOPIC_LOCAL_POSE, self.pose_cb, 10)
        self.labels_sub = self.create_subscription(String, TOPIC_LABELS, self.labels_cb, 10)
        self.dists_sub = self.create_subscription(Float32MultiArray, TOPIC_DISTS, self.dists_cb, 10)

        self.cur_x: Optional[float] = None
        self.cur_y: Optional[float] = None
        self.cur_yaw: Optional[float] = None  # [추가] 현재 Yaw 저장

        self.goal_x: Optional[float] = None
        self.goal_y: Optional[float] = None
        self.goal_yaw: Optional[float] = None # [추가] 목표 Yaw 저장

        self.last_labels: List[str] = []
        self.last_dists: List[float] = []

        self.mission_thread = threading.Thread(target=self.mission_logic)
        self.mission_thread.start()

        self.get_logger().info("Mission4Box started (XY + Yaw check).")

    # =========================
    # Callbacks
    # =========================
    def pose_cb(self, msg: PoseStamped):
        self.cur_x = msg.pose.position.x
        self.cur_y = msg.pose.position.y
        # [추가] 쿼터니언을 Yaw로 변환하여 저장
        self.cur_yaw = quaternion_to_yaw(msg.pose.orientation)

    def labels_cb(self, msg: String):
        s = msg.data.strip()
        self.last_labels = [] if (not s or s == "None") else [t.strip() for t in s.split(",") if t.strip()]

    def dists_cb(self, msg: Float32MultiArray):
        self.last_dists = list(msg.data)

    # =========================
    # Navigation helpers
    # =========================
    def publish_goal(self, x: float, y: float, yaw: float):
        g = PoseStamped()
        g.header.frame_id = "map"
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x = float(x)
        g.pose.position.y = float(y)
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        g.pose.orientation.x = qx
        g.pose.orientation.y = qy
        g.pose.orientation.z = qz
        g.pose.orientation.w = qw

        self.goal_x = float(x)
        self.goal_y = float(y)
        self.goal_yaw = float(yaw) # [추가]

        self.goal_pub.publish(g)
        self.get_logger().info(f"PUB GOAL: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")

    # [수정] 거리와 각도 모두 체크하는 함수
    def check_reached(self) -> bool:
        if self.cur_x is None or self.goal_x is None or self.cur_yaw is None:
            return False

        # 1. 거리 체크
        dist = math.hypot(self.goal_x - self.cur_x, self.goal_y - self.cur_y)
        
        # 2. 각도 체크 (중요: -PI ~ PI 경계 처리)
        yaw_diff = normalize_angle(self.goal_yaw - self.cur_yaw)
        yaw_err = abs(yaw_diff)

        # 디버깅 로그 (너무 자주 찍히면 주석 처리)
        # self.get_logger().info(f"DistErr: {dist:.2f}, YawErr: {yaw_err:.2f}")

        if dist <= GOAL_RADIUS and yaw_err <= GOAL_YAW_TOLERANCE:
            return True
        return False

    def wait_goal_reached(self, timeout_s: float) -> bool:
        t0 = time.time()
        while rclpy.ok():
            # [수정] check_reached() 사용
            if self.check_reached():
                self.get_logger().info("--> Reached Goal (XY & Yaw)!")
                return True
            
            if time.time() - t0 > timeout_s:
                self.get_logger().warn(f"Goal Timeout! (Wait: {timeout_s}s)")
                # 타임아웃 시 현재 오차 출력
                if self.cur_x is not None and self.goal_x is not None:
                    dist = math.hypot(self.goal_x - self.cur_x, self.goal_y - self.cur_y)
                    yaw_diff = abs(normalize_angle(self.goal_yaw - self.cur_yaw))
                    self.get_logger().warn(f"Final Error -> Dist: {dist:.2f}, Yaw: {yaw_diff:.2f}")
                return False
            
            time.sleep(0.1)
        return False

    def wait_pose_ready(self, timeout_s: float = 5.0) -> bool:
        t0 = time.time()
        while rclpy.ok():
            if self.cur_x is not None:
                return True
            if time.time() - t0 > timeout_s:
                self.get_logger().error("No pose received.")
                return False
            time.sleep(0.05)
        return False

    # =========================
    # Perception
    # =========================
    def get_nearest_box_dist(self) -> Optional[float]:
        labels = list(self.last_labels)
        dists = list(self.last_dists)
        n = min(len(labels), len(dists))
        if n == 0:
            return None
        best = None
        for i in range(n):
            if labels[i] == BOX_LABEL:
                d = float(dists[i])
                if d > 0.0 and (best is None or d < best):
                    best = d
        return best

    def box_present_check(self) -> bool:
        hits = 0
        for _ in range(SAMPLES):
            d = self.get_nearest_box_dist()
            if d is not None and 0.0 < d <= DIST_MAX:
                hits += 1
            time.sleep(SAMPLE_INTERVAL)
        self.get_logger().info(f"[Check] hits={hits}/{SAMPLES}")
        return hits >= MIN_HITS

    # =========================
    # Geometry Helper
    # =========================
    def yaw_box_to_goal(self, bx, by):
        return math.atan2(GOAL_Y - by, GOAL_X - bx)

    def obs_pose_from_box(self, bx, by, yaw):
        return (bx - math.cos(yaw) * OBS_OFFSET,
                by - math.sin(yaw) * OBS_OFFSET,
                yaw)

    def pre_pose_from_box(self, bx, by, yaw):
        return (bx - math.cos(yaw) * PRE_OFFSET,
                by - math.sin(yaw) * PRE_OFFSET,
                yaw)

    def push_goal_from_goal(self, yaw):
        return (GOAL_X - math.cos(yaw) * STOP_BEFORE_GOAL,
                GOAL_Y - math.sin(yaw) * STOP_BEFORE_GOAL,
                yaw)

    # =========================
    # Thread Logic
    # =========================
    def mission_logic(self):
        time.sleep(1.0)
        if not self.wait_pose_ready(6.0):
            return

        for cand in BOX_CANDIDATES:
            self.get_logger().info(f"=== Candidate: {cand['name']} ===")

            yaw = self.yaw_box_to_goal(cand["box_x"], cand["box_y"])
            
            # 1. Observe
            obs = self.obs_pose_from_box(cand["box_x"], cand["box_y"], yaw)
            self.publish_goal(*obs)
            if not self.wait_goal_reached(STEP_TIMEOUT):
                self.get_logger().warn("Failed to reach OBS pose.")
                continue # 다음 후보로
            time.sleep(SETTLE_TIME)

            # 2. Check Box
            if not self.box_present_check():
                self.get_logger().info("Box not found here.")
                continue

            # 3. Pre-push (Align)
            pre = self.pre_pose_from_box(cand["box_x"], cand["box_y"], yaw)
            self.publish_goal(*pre)
            if not self.wait_goal_reached(STEP_TIMEOUT):
                self.get_logger().warn("Failed to reach PRE pose.")
                continue
            time.sleep(SETTLE_TIME)

            # 4. Push
            push = self.push_goal_from_goal(yaw)
            self.publish_goal(*push)
            if not self.wait_goal_reached(STEP_TIMEOUT):
                 self.get_logger().warn("Failed to reach PUSH goal.")
                 # 실패했더라도 일단 루프 종료할지, 다음 박스 갈지 결정 필요. 여기선 종료
                 break

            self.get_logger().info("MISSION COMPLETE")
            return

        self.get_logger().error("All candidates failed or done.")


def main(args=None):
    rclpy.init(args=args)
    node = Mission4Box()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()