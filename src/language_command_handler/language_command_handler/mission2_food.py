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
# 20 room "entrance/door-front" observation start points (x, y, yaw_in)
# DO NOT DELETE COMMENTS
# =========================================================
ROOM_ENTRANCES: List[Tuple[float, float, float]] = [
    (-7.728, -4.406, 1.76),  # room5 left.
    (-1.507, -2.144, -1.46),  # room1 ml.
    (1.051, -2.094, -1.551),  # room1 mr.
    (8.180, -6.583, 1.416),  # room5 r.
    (-7.788, -18.861, 1.720),  # room6 left.
    (-8.213, -26.976, -1.600),  # nurse.
    (5.591, -27.694, -0.002),  # room7 r.
    (8.100, 14.372, 1.416),  # room2 r.
    (4.403, 17.164, 1.553),  # room1 r.
    (-4.552, 16.854, 1.516), # room1 left.
    (-7.853, 13.862, 1.626),  # room2 left.
    (-9.767, 10.942, -2.682),  # room3 left.
    (-9.650, 5.201, 3.031),  # room4 left.
    (8.116, -19.438, 1.551),  # room6 r.
    (1.720, -11.124, -1.664),  # room2 mr.
    (-1.881, -10.948, -1.304),  # room2 ml.
    (2.284, -17.144, -1.910),  # room3 mr.
    (-2.465, -16.843, -1.212),  # room3 ml.
    (10.297, 5.973, -0.100),  # room4 r
    (10.199, 10.288, -0.036),  # room3 r
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

SETTLE_TIME = 0.8

GOAL_RADIUS = 0.35
GOAL_YAW_TOLERANCE = 0.20

INROOM_STEPS = [0.0, 2.0 , 3.0]

# d-specific step timeouts (seconds)
# (너 코드에 2.0이 있었는데 실제 스텝은 1.5, 4.5라서 맞춰줌)
STEP_TIMEOUTS = {
    1.5: 15.0,
    2.0: 15.0,
    3.0: 15.0,
}
DEFAULT_STEP_TIMEOUT = 30.0

# 360 scan
SCAN_WZ = 0.22
SCAN_TOTAL_TIME = 20
MIN_HITS = 7

# 접근 모드에서는 타임아웃을 쓰지 않음 (good이면 bark까지 가야 하니까)
FORWARD_SPEED = 0.12
TURN_SPEED = 0.22

TOTAL_FOOD_TO_FINISH = 10  # log only

# wander / obstacle avoidance
FRONT_BLOCK_DIST = 0.55
SIDE_CHECK_DEG = 55
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
        self.last_good_yaw: Optional[float] = None

        self.food_seen = 0  # bad만 세기

        # bark가 예전에 남아있는 값이면 바로 종료되는 걸 막기 위해
        self.last_bark_time: float = -1.0

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
        s = msg.data.strip().lower()
        self.speech = s
        if s == "bark":
            self.last_bark_time = time.time()

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
    # Core behaviors
    # -------------------------
    def look_around(self):
        """
        Rotate in place for ~360 degrees (one full spin).
        Return: "good" / "bad" / "none"
        """
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
        """
        기존 코드는 good 1번만 떠도 바로 리턴이라 너무 약함.
        여기서는 MIN_HITS 연속으로 들어와야 good, bad 판정.
        """
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

    def approach_until_bark(self) -> bool:
        """
        규칙:
        - 이 함수가 호출된 순간은 good이 '확실'하게 잡힌 순간
        - 그 다음은 bark가 실제로 찍힐 때까지 절대 종료하지 않음
        - bad가 중간에 떠도 무시 (이미 good 확정이므로)
        - bark도 '접근 시작 이후'의 bark만 인정 (예전 bark 잔상 방지)
        """
        start_t = time.time()
        if self.last_good_yaw is not None:
            self._turn_toward(self.last_good_yaw, max_time=2.0)

        lost_t0 = None

        while rclpy.ok():
            # bark는 접근 시작 이후에 찍힌 것만 인정
            if self.last_bark_time > start_t:
                self.cmd_pub.publish(Twist())
                return True

            # good 계속 보이면 전진 우선
            if self.has_good():
                self.last_good_yaw = self.cur_yaw
                lost_t0 = None
                tw = self._avoidance_twist(FORWARD_SPEED)
                self.cmd_pub.publish(tw)
                time.sleep(0.1)
                continue

            # good을 잠깐 놓친 경우: 마지막 방향으로 돌거나, 제자리 탐색 회전
            if lost_t0 is None:
                lost_t0 = time.time()
            dt = time.time() - lost_t0

            if self._front_clear() < FRONT_BLOCK_DIST:
                tw = self._avoidance_twist(0.0)
                self.cmd_pub.publish(tw)
                time.sleep(0.1)
                continue

            tw = Twist()
            if dt < 2.0 and self.last_good_yaw is not None and self.cur_yaw is not None:
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
            # 1) Go to entrance
            self.publish_goal(ex, ey, yaw)
            self.wait_reached(ex, ey, yaw, timeout_s=None)
            time.sleep(SETTLE_TIME)

            # 2) 360 scan at entrance
            res0 = self.look_around()

            if res0 == "bad":
                self.food_seen += 1
                self.get_logger().info(
                    f"[room {idx}] BAD found at entrance total_bad={self.food_seen} -> next room"
                )
                continue

            if res0 == "good":
                self.get_logger().info(
                    f"[room {idx}] GOOD found at entrance -> MUST bark before end"
                )
                self.approach_until_bark()
                self.cmd_pub.publish(Twist())
                self.get_logger().info("BARK confirmed -> MISSION END")
                return

            # 3) In-room steps + wander
            goto_next_room = False

            for d in INROOM_STEPS:
                px = ex + math.cos(yaw) * d
                py = ey + math.sin(yaw) * d

                self.publish_goal(px, py, yaw)

                # d=0 : no timeout; d>0 : timeout; if timeout -> give up this room
                if d > 0.0:
                    timeout = STEP_TIMEOUTS.get(float(d), DEFAULT_STEP_TIMEOUT)
                    if not self.wait_reached(px, py, yaw, timeout_s=timeout):
                        self.get_logger().warn(
                            f"[room {idx}] Inroom reach timeout d={d} timeout={timeout}s -> GIVE UP ROOM"
                        )
                        goto_next_room = True
                        break
                    time.sleep(SETTLE_TIME)

                # 360 scan
                res = self.look_around()

                if res == "bad":
                    self.food_seen += 1
                    self.get_logger().info(
                        f"[room {idx}] BAD found in scan total_bad={self.food_seen} -> next room"
                    )
                    goto_next_room = True
                    break

                if res == "good":
                    self.get_logger().info(
                        f"[room {idx}] GOOD found in scan -> MUST bark before end"
                    )
                    self.approach_until_bark()
                    self.cmd_pub.publish(Twist())
                    self.get_logger().info("BARK confirmed -> MISSION END")
                    return

                # wander
                resw = self.wander_with_space(WANDER_TIME_PER_STEP)

                if resw == "bad":
                    self.food_seen += 1
                    self.get_logger().info(
                        f"[room {idx}] BAD found while wandering total_bad={self.food_seen} -> next room"
                    )
                    goto_next_room = True
                    break

                if resw == "good":
                    self.get_logger().info(
                        f"[room {idx}] GOOD found while wandering -> MUST bark before end"
                    )
                    self.approach_until_bark()
                    self.cmd_pub.publish(Twist())
                    self.get_logger().info("BARK confirmed -> MISSION END")
                    return

            if goto_next_room:
                continue

        self.get_logger().info("MISSION END: scanned all rooms, no GOOD found")
        self.cmd_pub.publish(Twist())

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