#!/usr/bin/env python3
import math
import time
import threading
from typing import List, Tuple, Optional

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import LaserScan

# =========================================================
# 20 room "entrance/door-front" observation start points (x, y, yaw_in)
# DO NOT DELETE COMMENTS
# =========================================================
ROOM_ENTRANCES: List[Tuple[float, float, float]] = [
    (-8.213, -26.976, -1.600),  # nurse.
    (-7.788, -18.861, 1.720),  # room6 left.
    (-7.728, -4.406, 1.76),  # room5 left.
    (-7.853, 13.862, 1.626),  # room2 left.
    (-4.552, 16.854, 1.516), # room1 left.
    (4.403, 17.164, 1.553),  # room1 r.
    (8.100, 14.372, 1.416),  # room2 r.
    (8.180, -6.583, 1.416),  # room5 r.
    (8.116, -19.438, 1.551),  # room6 r.
    (5.591, -27.694, -0.002),  # room7 r.
    (2.284, -17.144, -1.910),  # room3 mr.
    (-2.465, -16.843, -1.212),  # room3 ml.
    (-1.881, -10.948, -1.304),  # room2 ml.
    (1.720, -11.124, -1.664),  # room2 mr.
    (1.051, -2.094, -1.551),  # room1 mr.
    (-1.507, -2.144, -1.46),  # room1 ml.
    (-9.650, 5.201, 3.031),  # room4 left.
    (-9.767, 10.942, -2.682),  # room3 left.
    (10.199, 10.288, -0.036),  # room3 r
    (10.297, 5.973, -0.100),  # room4 r
]

# =========================================================
# ROS topics
# =========================================================
TOPIC_GOAL = "/goal_pose"
TOPIC_CMD = "/cmd_vel"
TOPIC_LOCAL_POSE = "/go1_pose"
TOPIC_LABELS = "/detections/labels"
TOPIC_CENTERS = "/detections/centers"
TOPIC_SPEECH = "/robot_dog/speech"
TOPIC_SCAN = "/scan"

# =========================================================
# Navigation / goal reaching
# =========================================================
SETTLE_TIME = 0.8
GOAL_RADIUS = 0.35
GOAL_YAW_TOLERANCE = 0.20

INROOM_STEPS = [1.0, 2.0, 3.0]
STEP_TIMEOUTS = {1.0: 15.0, 2.0: 15.0, 3.0: 15.0}
DEFAULT_STEP_TIMEOUT = 30.0

# =========================================================
# Vision-based approach params (nurse-style)
# =========================================================
IMG_WIDTH = 640
FOV_DEG = 80.0
FOCAL_LENGTH = (IMG_WIDTH / 2) / math.tan(math.radians(FOV_DEG / 2))

ALIGN_TOL = 0.04             # rad
TARGET_DIST = 0.9            # m (front lidar target distance)
SEARCH_WZ = 0.30             # rad/s (rotate when lost)
VISION_APPROACH_TIMEOUT = 90.0  # s (safety, but "아깝게 포기" 줄이려고 크게)

# =========================================================
# 360 scan / confirmation
# =========================================================
SCAN_WZ = 0.22
SCAN_TOTAL_TIME = 20
MIN_HITS = 5

# =========================================================
# Obstacle avoidance
# =========================================================
FORWARD_SPEED = 0.12
TURN_SPEED = 0.22
FRONT_BLOCK_DIST = 0.55
SIDE_CHECK_DEG = 55
WANDER_TIME_PER_STEP = 4.0

# =========================================================
# Utils
# =========================================================
def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def quaternion_to_yaw(q):
    return math.atan2(
        2 * (q.w * q.z + q.x * q.y),
        1 - 2 * (q.y * q.y + q.z * q.z),
    )


def normalize_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def parse_labels(s: str):
    if not s or s == "None":
        return []
    return [t.strip().lower() for t in s.split(",") if t.strip()]


# =========================================================
# Mission Node
# =========================================================
class Mission2Food(Node):
    def __init__(self):
        super().__init__("mission2_food")

        self.goal_pub = self.create_publisher(PoseStamped, TOPIC_GOAL, 10)
        self.cmd_pub = self.create_publisher(Twist, TOPIC_CMD, 10)

        self.create_subscription(PoseStamped, TOPIC_LOCAL_POSE, self.pose_cb, 10)
        self.create_subscription(String, TOPIC_LABELS, self.labels_cb, 10)
        self.create_subscription(Float32MultiArray, TOPIC_CENTERS, self.centers_cb, 10)
        self.create_subscription(String, TOPIC_SPEECH, self.speech_cb, 10)
        self.create_subscription(LaserScan, TOPIC_SCAN, self.scan_cb, 10)

        # robot pose
        self.cur_x: Optional[float] = None
        self.cur_y: Optional[float] = None
        self.cur_yaw: Optional[float] = None

        # perception
        self.labels: List[str] = []
        self.good_found: bool = False
        self.good_idx: int = -1
        self.good_cx: float = -1.0
        self.last_seen_time: float = 0.0

        # bark tracking (prevent old bark)
        self.last_bark_time: float = -1.0

        # scan
        self.scan_msg: Optional[LaserScan] = None

        # memory for scan-based confirmation
        self.last_good_yaw: Optional[float] = None

        # stats
        self.bad_seen = 0

        threading.Thread(target=self.mission_logic, daemon=True).start()

    # -------------------------
    # Callbacks
    # -------------------------
    def pose_cb(self, msg: PoseStamped):
        self.cur_x = msg.pose.position.x
        self.cur_y = msg.pose.position.y
        self.cur_yaw = quaternion_to_yaw(msg.pose.orientation)

    def labels_cb(self, msg: String):
        self.labels = parse_labels(msg.data)

        self.good_found = False
        self.good_idx = -1
        for i, t in enumerate(self.labels):
            if "good" in t:
                self.good_found = True
                self.good_idx = i
                self.last_seen_time = time.time()
                break

    def centers_cb(self, msg: Float32MultiArray):
        if self.good_found and 0 <= self.good_idx < len(msg.data):
            self.good_cx = float(msg.data[self.good_idx])

    def speech_cb(self, msg: String):
        s = msg.data.strip().lower()
        if s == "bark":
            self.last_bark_time = time.time()

    def scan_cb(self, msg: LaserScan):
        self.scan_msg = msg

    # -------------------------
    # Navigation helpers
    # -------------------------
    def publish_goal(self, x, y, yaw):
        g = PoseStamped()
        g.header.frame_id = "map"
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x = float(x)
        g.pose.position.y = float(y)
        q = yaw_to_quaternion(yaw)
        g.pose.orientation.z = q[2]
        g.pose.orientation.w = q[3]
        self.goal_pub.publish(g)

    def wait_reached(self, x, y, yaw, timeout_s=None) -> bool:
        t0 = time.time()
        while rclpy.ok():
            if self.cur_x is None or self.cur_yaw is None:
                time.sleep(0.05)
                continue

            dist = math.hypot(x - self.cur_x, y - self.cur_y)
            yaw_err = abs(normalize_angle(yaw - self.cur_yaw))

            if dist <= GOAL_RADIUS and yaw_err <= GOAL_YAW_TOLERANCE:
                return True

            if timeout_s is not None and (time.time() - t0) > timeout_s:
                return False

            time.sleep(0.05)
        return False

    # -------------------------
    # Scan-based free-space motion
    # -------------------------
    def _range_at_angle(self, ang_rad: float) -> float:
        msg = self.scan_msg
        if msg is None or len(msg.ranges) == 0:
            return 999.0
        i = int(round((ang_rad - msg.angle_min) / msg.angle_increment))
        i = max(0, min(i, len(msg.ranges) - 1))
        r = msg.ranges[i]
        if math.isfinite(r) and r > 0.0:
            return r
        return 999.0

    def _front_clear(self) -> float:
        a = math.radians(18)
        vals = [self._range_at_angle(0.0), self._range_at_angle(+a), self._range_at_angle(-a)]
        return min(vals)

    def _left_clear(self) -> float:
        return self._range_at_angle(math.radians(SIDE_CHECK_DEG))

    def _right_clear(self) -> float:
        return self._range_at_angle(-math.radians(SIDE_CHECK_DEG))

    def _avoidance_twist(self, forward_speed: float) -> Twist:
        tw = Twist()
        front = self._front_clear()
        left = self._left_clear()
        right = self._right_clear()

        if front < FRONT_BLOCK_DIST:
            tw.linear.x = 0.0
            tw.angular.z = +TURN_SPEED if left > right else -TURN_SPEED
            return tw

        tw.linear.x = forward_speed
        bias = max(-0.25, min(0.25, (left - right) * 0.15))
        tw.angular.z = bias
        return tw

    # -------------------------
    # 360 scan confirmation (good/bad/none)
    # -------------------------
    def has_good(self) -> bool:
        return any("good" in t for t in self.labels)

    def has_bad(self) -> bool:
        return any("bad" in t for t in self.labels)

    def look_around(self) -> str:
        if self.cur_yaw is None:
            return "none"

        turned = 0.0
        last_yaw = self.cur_yaw
        t0 = time.time()

        good_hits = 0
        bad_hits = 0

        tw = Twist()
        tw.angular.z = +SCAN_WZ

        while rclpy.ok():
            if self.has_good():
                good_hits += 1
                self.last_good_yaw = self.cur_yaw
            else:
                good_hits = 0

            if self.has_bad():
                bad_hits += 1
            else:
                bad_hits = 0

            if good_hits >= MIN_HITS:
                self.cmd_pub.publish(Twist())
                return "good"
            if bad_hits >= MIN_HITS:
                self.cmd_pub.publish(Twist())
                return "bad"

            cur = self.cur_yaw
            d = normalize_angle(cur - last_yaw)
            turned += abs(d)
            last_yaw = cur

            if turned >= 2.0 * math.pi:
                break
            if time.time() - t0 > SCAN_TOTAL_TIME:
                break

            self.cmd_pub.publish(tw)
            time.sleep(0.05)

        self.cmd_pub.publish(Twist())
        return "none"

    def wander_with_space(self, duration_s: float) -> str:
        t0 = time.time()
        good_hits = 0
        bad_hits = 0

        while time.time() - t0 < duration_s and rclpy.ok():
            if self.has_good():
                good_hits += 1
                self.last_good_yaw = self.cur_yaw
            else:
                good_hits = 0

            if self.has_bad():
                bad_hits += 1
            else:
                bad_hits = 0

            if good_hits >= MIN_HITS:
                self.cmd_pub.publish(Twist())
                return "good"
            if bad_hits >= MIN_HITS:
                self.cmd_pub.publish(Twist())
                return "bad"

            self.cmd_pub.publish(self._avoidance_twist(FORWARD_SPEED))
            time.sleep(0.1)

        self.cmd_pub.publish(Twist())
        return "none"

    # -------------------------
    # Vision-based alignment helpers (nurse-style)
    # -------------------------
    def good_angle_error(self) -> float:
        if self.good_cx <= 0:
            return 0.0
        pixel_error = (IMG_WIDTH / 2) - self.good_cx
        return float(math.atan2(pixel_error, FOCAL_LENGTH))

    def vision_approach_to_bark(self) -> bool:
        """
        nurse처럼 good의 cx로 정렬하고, 라이다 전방거리로 목표거리까지 접근.
        bark가 찍히면 즉시 정지 + True.
        good을 잠깐 놓쳐도 회전 탐색으로 재획득해서 계속 시도.
        """
        start_t = time.time()

        while rclpy.ok():
            # bark 확인(접근 시작 이후)
            if self.last_bark_time > start_t:
                self.cmd_pub.publish(Twist())  # 즉시 멈춤
                return True

            # 안전장치 (원하면 더 키워도 됨)
            if time.time() - start_t > VISION_APPROACH_TIMEOUT:
                self.cmd_pub.publish(Twist())
                return False

            # good 못 보면 제자리 회전 탐색
            if not self.good_found or self.good_cx <= 0:
                tw = Twist()
                tw.angular.z = SEARCH_WZ
                self.cmd_pub.publish(tw)
                time.sleep(0.1)
                continue

            # 정렬
            err = self.good_angle_error()
            if abs(err) > ALIGN_TOL:
                tw = Twist()
                tw.angular.z = max(-0.5, min(0.5, 1.6 * err))
                self.cmd_pub.publish(tw)
                time.sleep(0.05)
                continue

            # 접근 (전방 라이다)
            dist = self._front_clear()
            tw = Twist()

            if dist > TARGET_DIST + 0.10:
                tw.linear.x = 0.22
            elif dist < TARGET_DIST - 0.10:
                tw.linear.x = -0.10
            else:
                # 목표 근처: 천천히 전진하면서 bark 유도
                tw.linear.x = 0.08

            # 접근 중에도 미세 정렬
            tw.angular.z = max(-0.35, min(0.35, 1.2 * err))

            # 장애물 너무 가까우면 멈추고 회전(충돌 방지)
            if dist < 0.35:
                tw.linear.x = 0.0
                tw.angular.z = TURN_SPEED

            self.cmd_pub.publish(tw)
            time.sleep(0.08)

        self.cmd_pub.publish(Twist())
        return False

    # -------------------------
    # Mission logic
    # -------------------------
    def mission_logic(self):
        time.sleep(1.0)

        for idx, (ex, ey, yaw) in enumerate(ROOM_ENTRANCES, 1):
            # 1) Go to entrance
            self.publish_goal(ex, ey, yaw)
            self.wait_reached(ex, ey, yaw, timeout_s=None)
            time.sleep(SETTLE_TIME)

            # 2) 360 scan at entrance
            res0 = self.look_around()

            if res0 == "bad":
                self.bad_seen += 1
                self.get_logger().info(f"[room {idx}] BAD found at entrance total_bad={self.bad_seen} -> next room")
                continue

            if res0 == "good":
                self.get_logger().info(f"[room {idx}] GOOD found -> vision-align approach until bark")
                if self.vision_approach_to_bark():
                    self.get_logger().info("BARK confirmed -> STOP and END")
                    self.cmd_pub.publish(Twist())
                    return
                else:
                    self.get_logger().warn("BARK not confirmed -> resume searching in room")

            # 3) In-room steps + wander
            goto_next_room = False

            for d in INROOM_STEPS:
                px = ex + math.cos(yaw) * d
                py = ey + math.sin(yaw) * d

                self.publish_goal(px, py, yaw)

                timeout = STEP_TIMEOUTS.get(float(d), DEFAULT_STEP_TIMEOUT)
                if not self.wait_reached(px, py, yaw, timeout_s=timeout):
                    self.get_logger().warn(f"[room {idx}] Inroom reach timeout d={d} -> GIVE UP ROOM")
                    goto_next_room = True
                    break
                time.sleep(SETTLE_TIME)

                # 360 scan
                res = self.look_around()

                if res == "bad":
                    self.bad_seen += 1
                    self.get_logger().info(f"[room {idx}] BAD found in scan total_bad={self.bad_seen} -> next room")
                    goto_next_room = True
                    break

                if res == "good":
                    self.get_logger().info(f"[room {idx}] GOOD found -> vision-align approach until bark")
                    if self.vision_approach_to_bark():
                        self.get_logger().info("BARK confirmed -> STOP and END")
                        self.cmd_pub.publish(Twist())
                        return
                    else:
                        self.get_logger().warn("BARK not confirmed -> resume searching")

                # wander
                resw = self.wander_with_space(WANDER_TIME_PER_STEP)

                if resw == "bad":
                    self.bad_seen += 1
                    self.get_logger().info(f"[room {idx}] BAD found while wandering total_bad={self.bad_seen} -> next room")
                    goto_next_room = True
                    break

                if resw == "good":
                    self.get_logger().info(f"[room {idx}] GOOD found while wandering -> vision-align approach until bark")
                    if self.vision_approach_to_bark():
                        self.get_logger().info("BARK confirmed -> STOP and END")
                        self.cmd_pub.publish(Twist())
                        return
                    else:
                        self.get_logger().warn("BARK not confirmed -> resume searching")

            if goto_next_room:
                continue

        self.get_logger().info("MISSION END: scanned all rooms, no GOOD found")
        self.cmd_pub.publish(Twist())


def main():
    rclpy.init()
    node = Mission2Food()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
