#!/usr/bin/env python3
import math
import time
from typing import Optional, Dict, List

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String, Float32

# =========================
# FIXED GOAL (red area center)  <-- 너 맵 값으로 수정 필수
# =========================
GOAL_X = 0.0
GOAL_Y = 12.0

# goal 중심까지 로봇이 "덜 가게" 만드는 오프셋 (미는 중 박스가 골 밖으로 나가는 리스크 완화)
# goal 방향 반대쪽으로 이만큼 떨어진 지점을 push 목표로 삼음
STOP_BEFORE_GOAL = 0.3   # 0.3~0.6 사이에서 튜닝 추천

# =========================
# BOX CANDIDATES (박스 중심 좌표 3개) <-- 맵 값으로 수정 필수
# TA가 3개 후보 중 하나에 둔다고 했으니, 후보 좌표는 고정으로 둠
# =========================
BOX_CANDIDATES: List[Dict[str, float]] = [
    {"name": "cand1", "box_x": 0.0, "box_y": 14.0},
    {"name": "cand2", "box_x": 2.0, "box_y": 12.0},
    {"name": "cand3", "box_x": -2.0, "box_y": 12.0},
]

# =========================
# Perception topics (네 perception_node.py 그대로)
# =========================
TOPIC_LABEL = "/detections/labels"
TOPIC_DIST  = "/detections/distance"   # 선택적으로 판정 강화할 때 사용

# label 가정: 학습하면 "box"가 나온다고 가정
BOX_LABEL = "box"

# 거리 게이트(너무 멀리 보이는 걸 후보 박스로 착각하는 것 방지)
# depth가 0.0이면 (NaN 처리 결과) 신뢰 낮으니 제외
DIST_MIN = 0.25
DIST_MAX = 3.0

# 관찰 위치에서 샘플링으로 안정화
SAMPLES = 10
SAMPLE_INTERVAL = 0.08
MIN_HITS = 2

# =========================
# Navigation / timing
# =========================
STEP_TIMEOUT = 35.0
SETTLE_TIME = 0.4

# =========================
# Geometry offsets (박스 기준으로 obs/pre 위치 자동 생성)
# =========================
OBS_OFFSET = 1.2    # 박스 뒤쪽(밀기 방향 반대쪽)에서 관찰할 거리
PRE_OFFSET = 0.75   # 박스 바로 뒤(밀기 시작 위치)


def yaw_to_quaternion(yaw: float):
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return qx, qy, qz, qw


class Mission4Box(Node):
    def __init__(self):
        super().__init__("mission4_box")

        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)

        self.goal_reached = False
        self.goal_sent = False
        self.finished = False
        self.create_subscription(Bool, "/goal_reached", self.goal_reached_cb, 10)

        self.last_label: str = "None"
        self.last_dist: float = 0.0
        self.create_subscription(String, TOPIC_LABEL, self.label_cb, 10)
        self.create_subscription(Float32, TOPIC_DIST, self.dist_cb, 10)

        self.timer = self.create_timer(0.8, self.run_once)
        self.started = False

        self.get_logger().info("Mission 4 started (fixed goal, candidate scan, perception-based).")

    # ---------- callbacks ----------
    def goal_reached_cb(self, msg: Bool):
        if self.finished:
            return
        if msg.data and self.goal_sent:
            self.goal_reached = True
            self.get_logger().info(">>> /goal_reached = True")

    def label_cb(self, msg: String):
        self.last_label = msg.data.strip()

    def dist_cb(self, msg: Float32):
        self.last_dist = float(msg.data)

    # ---------- nav helpers ----------
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

        self.goal_reached = False
        self.goal_sent = True
        self.goal_pub.publish(g)
        self.get_logger().info(f"Publish /goal_pose: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")

    def wait_goal_reached(self, timeout_s: float) -> bool:
        t0 = time.time()
        while rclpy.ok():
            if self.goal_reached:
                return True
            if time.time() - t0 > timeout_s:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    # ---------- perception check ----------
    def box_hit_now(self) -> bool:
        if self.last_label.lower() != BOX_LABEL:
            return False
        # distance가 0이면(깊이 이상값) 신뢰 낮으니 제외
        if not (DIST_MIN <= self.last_dist <= DIST_MAX):
            return False
        return True

    def box_present_check(self) -> bool:
        hits = 0
        for _ in range(SAMPLES):
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.box_hit_now():
                hits += 1
            time.sleep(SAMPLE_INTERVAL)

        self.get_logger().info(
            f"Perception: hits={hits}/{SAMPLES}, label={self.last_label}, dist={self.last_dist:.2f}"
        )
        return hits >= MIN_HITS

    # ---------- geometry ----------
    def yaw_box_to_goal(self, box_x: float, box_y: float) -> float:
        return math.atan2(GOAL_Y - box_y, GOAL_X - box_x)

    def obs_pose_from_box(self, box_x: float, box_y: float, yaw: float):
        # 박스 뒤쪽(밀기 방향 반대쪽)으로 OBS_OFFSET 떨어진 점
        ox = box_x - math.cos(yaw) * OBS_OFFSET
        oy = box_y - math.sin(yaw) * OBS_OFFSET
        return ox, oy, yaw

    def pre_pose_from_box(self, box_x: float, box_y: float, yaw: float):
        # 박스 뒤쪽(밀기 시작 위치)으로 PRE_OFFSET 떨어진 점
        px = box_x - math.cos(yaw) * PRE_OFFSET
        py = box_y - math.sin(yaw) * PRE_OFFSET
        return px, py, yaw

    def push_goal_from_goal(self, yaw: float):
        # goal 중심까지 가지 말고, goal 중심 "앞쪽"에서 멈추게 만들기
        gx = GOAL_X - math.cos(yaw) * STOP_BEFORE_GOAL
        gy = GOAL_Y - math.sin(yaw) * STOP_BEFORE_GOAL
        return gx, gy, yaw

    # ---------- finish ----------
    def finish(self, reason: str):
        self.finished = True
        self.get_logger().info(f"Bark! Mission 4 complete. ({reason})")
        raise SystemExit

    # ---------- main ----------
    def run_once(self):
        if self.started:
            return
        self.started = True
        self.timer.cancel()

        # 후보 3개 순회
        for cand in BOX_CANDIDATES:
            name = cand["name"]
            box_x, box_y = cand["box_x"], cand["box_y"]

            yaw = self.yaw_box_to_goal(box_x, box_y)
            obs_x, obs_y, obs_yaw = self.obs_pose_from_box(box_x, box_y, yaw)
            pre_x, pre_y, pre_yaw = self.pre_pose_from_box(box_x, box_y, yaw)
            push_x, push_y, push_yaw = self.push_goal_from_goal(yaw)

            self.get_logger().info(f"=== Try {name} ===")

            # 1) 관찰 위치 이동
            self.publish_goal(obs_x, obs_y, obs_yaw)
            if not self.wait_goal_reached(STEP_TIMEOUT):
                self.get_logger().warn("obs timeout -> next")
                continue
            time.sleep(SETTLE_TIME)

            # 2) 박스 여부 판단 (label=box + 거리 범위 + 샘플링)
            if not self.box_present_check():
                self.get_logger().info("No box here -> next candidate.")
                continue

            self.get_logger().info("Box detected -> go pre-push.")

            # 3) 박스 뒤 정렬 위치로 이동
            self.publish_goal(pre_x, pre_y, pre_yaw)
            if not self.wait_goal_reached(STEP_TIMEOUT):
                self.get_logger().warn("pre timeout -> next")
                continue
            time.sleep(SETTLE_TIME)

            # 4) goal 방향으로 밀기 (goal 중심까지 가지 않고 살짝 덜 감)
            self.publish_goal(push_x, push_y, push_yaw)
            if not self.wait_goal_reached(STEP_TIMEOUT):
                self.get_logger().warn("push timeout -> next")
                continue

            self.finish(reason=f"{name}: pushed toward fixed goal")

        self.get_logger().error("All candidates tried, failed.")
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = Mission4Box()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
