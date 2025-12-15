#!/usr/bin/env python3
import math
import time
import threading
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

# =========================================================
# 방 20개 "입구/문 앞" 관찰 시작점 (x, y, yaw_in)
# 주석 지우지 마!
# =========================================================
ROOM_ENTRANCES: List[Tuple[float, float, float]] = [
    (-3.960, 16.2, 1.592),  # room1 left
    (-7.392, 12.704, 1.576),  # room2 left
    (-9.715, 10.548, -3.087),  # room3 left
    (-9.650, 5.201, 3.031),  # room4 left
    (-7.736, -9.006, 3.127),  # room5 left
    (-7.273, -22.891, 3.040),  # room6 left
    (-7.655, -25.973, -3.033),  # nurse
    (5.068, -27.260, -1.724),  # room7 r
    (7.324, -22.737, 0),  # room6 r
    (2.415, -16.745, -1.956),  # room3 mr
    (-3.101, -16.583, -0.917),  # room3 ml
    (2.342, -10.563, -2.256),  # room2 mr
    (-2.270, -10.874, -1.236),  # room2 ml
    (7.588, -8.933, 0),  # room5 r
    (2.244, -0.516, -1.967),  # room1 ml
    (-2.771, -0.651, -0.736),  # room1 mr
    (9.452, 4.863, 0.427),  # room4 r
    (9.377, 11.041, -0.278),  # room3 r
    (7.111, 13.244, 1.413),  # room2 r
    (4.921, 15.460, 1.528),  # room1 r
]

# =========================================================
# Constants
# =========================================================
TOPIC_GOAL = "/goal_pose"
TOPIC_CMD = "/cmd_vel"
TOPIC_LOCAL_POSE = "/go1_pose"
TOPIC_LABELS = "/detections/labels"
TOPIC_SPEECH = "/robot_dog/speech"
TOPIC_SCAN = "/scan"

STEP_TIMEOUT = 30.0
SETTLE_TIME = 0.8

GOAL_RADIUS = 0.35
GOAL_YAW_TOLERANCE = 0.20

INROOM_STEPS = [0.0, 1.0, 1.8]

# 회전 과하게 안 하게
SCAN_ANGLE_RANGE = math.radians(130)   # ±35도
SCAN_STEP_ANGLE = math.radians(6)
SCAN_WZ = 0.22
SCAN_TOTAL_TIME = 8.0

MIN_HITS = 2

APPROACH_TIMEOUT = 30.0
FORWARD_SPEED = 0.12
TURN_SPEED = 0.22

TOTAL_FOOD_TO_FINISH = 10

# 빈공간으로 움직이기용
FRONT_BLOCK_DIST = 0.55   # 앞이 이보다 가까우면 회피
SIDE_CHECK_DEG = 55       # 좌우 확인 각도
WANDER_TIME_PER_STEP = 4.0


# =========================================================
# Utility functions
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
    return [t.strip().lower() for t in s.split(",")]


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
        self.create_subscription(String, TOPIC_SPEECH, self.speech_cb, 10)
        self.create_subscription(LaserScan, TOPIC_SCAN, self.scan_cb, 10)

        self.cur_x = self.cur_y = self.cur_yaw = None
        self.labels = []
        self.speech = "None"

        self.scan_msg: Optional[LaserScan] = None
        self.last_good_yaw: Optional[float] = None  # good를 본 방향(대략)

        self.food_seen = 0

        threading.Thread(target=self.mission_logic, daemon=True).start()

    # -------------------------
    # Callbacks
    # -------------------------
    def pose_cb(self, msg):
        self.cur_x = msg.pose.position.x
        self.cur_y = msg.pose.position.y
        self.cur_yaw = quaternion_to_yaw(msg.pose.orientation)

    def labels_cb(self, msg):
        self.labels = parse_labels(msg.data)

    def speech_cb(self, msg):
        self.speech = msg.data.strip().lower()

    def scan_cb(self, msg: LaserScan):
        self.scan_msg = msg

    # -------------------------
    # Helpers
    # -------------------------
    def wait_reached(self, x, y, yaw, timeout_s=None):
        t0 = time.time()
        while rclpy.ok():
            if self.cur_x is None:
                time.sleep(0.05)
                continue

            dx = x - self.cur_x
            dy = y - self.cur_y
            dist = math.hypot(dx, dy)
            yaw_err = abs(normalize_angle(yaw - self.cur_yaw))

            if dist <= GOAL_RADIUS and yaw_err <= GOAL_YAW_TOLERANCE:
                return True

            if timeout_s is not None and (time.time() - t0) > timeout_s:
                return False

            time.sleep(0.05)
        return False

    def has_good(self):
        return any("good" in t for t in self.labels)

    def has_bad(self):
        return any("bad" in t for t in self.labels)

    # -------------------------
    # Scan 기반 빈공간 방향 선택
    # -------------------------
    def _range_at_angle(self, ang_rad: float) -> float:
        """
        로봇 기준 각도(라디안)에서 거리 값을 대략 가져옴
        scan이 없으면 큰 값 반환
        """
        msg = self.scan_msg
        if msg is None or len(msg.ranges) == 0:
            return 999.0

        # 각도 -> 인덱스
        # msg.angle_min + i*inc = ang
        i = int(round((ang_rad - msg.angle_min) / msg.angle_increment))
        i = max(0, min(i, len(msg.ranges) - 1))

        r = msg.ranges[i]
        if math.isfinite(r) and r > 0.0:
            return r
        return 999.0

    def _front_clear(self) -> float:
        # 정면 근처 여러 샘플 중 최솟값
        a = math.radians(18)
        vals = [
            self._range_at_angle(0.0),
            self._range_at_angle(+a),
            self._range_at_angle(-a),
        ]
        return min(vals)

    def _left_clear(self) -> float:
        ang = math.radians(SIDE_CHECK_DEG)
        return self._range_at_angle(+ang)

    def _right_clear(self) -> float:
        ang = math.radians(SIDE_CHECK_DEG)
        return self._range_at_angle(-ang)

    def _avoidance_twist(self, forward_speed: float) -> Twist:
        """
        앞이 막히면 더 넓은 쪽으로 돌고,
        앞이 비면 전진(약간의 보정)
        """
        tw = Twist()
        front = self._front_clear()
        left = self._left_clear()
        right = self._right_clear()

        if front < FRONT_BLOCK_DIST:
            # 더 넓은 쪽으로 회전
            tw.linear.x = 0.0
            tw.angular.z = +TURN_SPEED if left > right else -TURN_SPEED
            return tw

        # 앞이 비면 전진, 양쪽 차이가 크면 약간 틀기
        tw.linear.x = forward_speed
        bias = max(-0.25, min(0.25, (left - right) * 0.15))
        tw.angular.z = bias
        return tw

    # -------------------------
    # Core behaviors
    # -------------------------
    def look_around(self):
        start_yaw = self.cur_yaw
        target = start_yaw
        direction = 1
        t0 = time.time()

        good_hits = 0
        bad_hits = 0

        while time.time() - t0 < SCAN_TOTAL_TIME:
            if self.has_good():
                good_hits += 1
                self.last_good_yaw = self.cur_yaw  # good를 본 대략 방향 저장
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

            target = normalize_angle(target + direction * SCAN_STEP_ANGLE)
            if abs(normalize_angle(target - start_yaw)) > SCAN_ANGLE_RANGE:
                direction *= -1

            err = normalize_angle(target - self.cur_yaw)
            tw = Twist()
            tw.angular.z = max(-SCAN_WZ, min(SCAN_WZ, 1.0 * err))
            self.cmd_pub.publish(tw)
            time.sleep(0.05)

        self.cmd_pub.publish(Twist())
        return "none"

    def wander_with_space(self, duration_s: float) -> str:
        """
        빈공간으로 조금 돌아다니며 탐색
        good/bad가 보이면 즉시 반환
        """
        t0 = time.time()
        while time.time() - t0 < duration_s and rclpy.ok():
            if self.has_good():
                self.last_good_yaw = self.cur_yaw
                self.cmd_pub.publish(Twist())
                return "good"
            if self.has_bad():
                self.cmd_pub.publish(Twist())
                return "bad"

            tw = self._avoidance_twist(FORWARD_SPEED)
            self.cmd_pub.publish(tw)
            time.sleep(0.1)

        self.cmd_pub.publish(Twist())
        return "none"

    def _turn_toward(self, target_yaw: float, max_time: float = 3.0):
        t0 = time.time()
        while time.time() - t0 < max_time and rclpy.ok():
            if self.cur_yaw is None:
                time.sleep(0.05)
                continue
            err = normalize_angle(target_yaw - self.cur_yaw)
            if abs(err) < 0.12:
                break
            tw = Twist()
            tw.angular.z = max(-SCAN_WZ, min(SCAN_WZ, 1.2 * err))
            self.cmd_pub.publish(tw)
            time.sleep(0.05)
        self.cmd_pub.publish(Twist())

    def approach_until_bark(self):
        """
        good이면 그쪽으로 가서 bark
        - good를 멀리서라도 봤으면(last_good_yaw) 그쪽으로 먼저 틀고 접근
        - 앞이 막히면 빈공간으로 회피하며 접근
        - bad 나오면 바로 중단
        """
        # good를 본 방향이 있으면 우선 그쪽으로 살짝 맞추기
        if self.last_good_yaw is not None:
            self._turn_toward(self.last_good_yaw, max_time=2.0)

        t0 = time.time()
        while time.time() - t0 < APPROACH_TIMEOUT and rclpy.ok():
            if self.speech == "bark":
                self.cmd_pub.publish(Twist())
                return True
            if self.has_bad():
                self.cmd_pub.publish(Twist())
                return False

            # good 보이면 전진 위주, 안 보이면 last_good_yaw 쪽으로 천천히 돌면서 찾기
            if self.has_good():
                self.last_good_yaw = self.cur_yaw
                tw = self._avoidance_twist(FORWARD_SPEED)
                self.cmd_pub.publish(tw)
            else:
                tw = Twist()
                # 앞이 너무 막히면 회피 우선
                if self._front_clear() < FRONT_BLOCK_DIST:
                    tw = self._avoidance_twist(0.0)
                else:
                    if self.last_good_yaw is not None:
                        err = normalize_angle(self.last_good_yaw - self.cur_yaw)
                        tw.angular.z = max(-TURN_SPEED, min(TURN_SPEED, 1.0 * err))
                    else:
                        tw.angular.z = TURN_SPEED
                self.cmd_pub.publish(tw)

            time.sleep(0.1)

        self.cmd_pub.publish(Twist())
        return False

    # -------------------------
    # Mission logic
    # -------------------------
    def mission_logic(self):
        time.sleep(1.0)

        for idx, (ex, ey, yaw) in enumerate(ROOM_ENTRANCES, 1):
            if self.food_seen >= TOTAL_FOOD_TO_FINISH:
                self.get_logger().info("MISSION COMPLETE: found 10 foods")
                return

            # 1) 입구는 제한 없이 도달할 때까지
            self.publish_goal(ex, ey, yaw)
            self.wait_reached(ex, ey, yaw, timeout_s=None)
            time.sleep(SETTLE_TIME)

            room_done = False

            # 2) 입구에서 먼저 스캔
            res0 = self.look_around()
            if res0 == "bad":
                self.food_seen += 1
                self.get_logger().info(f"Food found (bad) total={self.food_seen} -> next room")
                continue
            if res0 == "good":
                self.food_seen += 1
                self.get_logger().info(f"Food found (good) total={self.food_seen} -> approach+bark")
                self.approach_until_bark()
                continue

            # 3) 안 보이면 inroom step + 빈공간으로 조금 돌아다니기
            for d in INROOM_STEPS:
                px = ex + math.cos(yaw) * d
                py = ey + math.sin(yaw) * d

                self.publish_goal(px, py, yaw)

                # 전진(d>0)만 시간 제한, 막히면 다음 방
                if d > 0.0:
                    if not self.wait_reached(px, py, yaw, timeout_s=STEP_TIMEOUT):
                        self.get_logger().warn(f"Inroom reach timeout: room idx={idx}, d={d} -> next room")
                        room_done = True
                        break
                    time.sleep(SETTLE_TIME)

                # 그 지점에서 스캔
                res = self.look_around()
                if res == "bad":
                    self.food_seen += 1
                    self.get_logger().info(f"Food found (bad) total={self.food_seen} -> next room")
                    room_done = True
                    break
                if res == "good":
                    self.food_seen += 1
                    self.get_logger().info(f"Food found (good) total={self.food_seen} -> approach+bark")
                    self.approach_until_bark()
                    room_done = True
                    break

                # 스캔에도 없으면, 빈공간으로 잠깐 돌아다니며 찾기
                resw = self.wander_with_space(WANDER_TIME_PER_STEP)
                if resw == "bad":
                    self.food_seen += 1
                    self.get_logger().info(f"Food found (bad) total={self.food_seen} -> next room")
                    room_done = True
                    break
                if resw == "good":
                    self.food_seen += 1
                    self.get_logger().info(f"Food found (good) total={self.food_seen} -> approach+bark")
                    self.approach_until_bark()
                    room_done = True
                    break

            if room_done:
                continue

    def publish_goal(self, x, y, yaw):
        g = PoseStamped()
        g.header.frame_id = "map"
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x = x
        g.pose.position.y = y
        q = yaw_to_quaternion(yaw)
        g.pose.orientation.z = q[2]
        g.pose.orientation.w = q[3]
        self.goal_pub.publish(g)


def main():
    rclpy.init()
    node = Mission2Food()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
