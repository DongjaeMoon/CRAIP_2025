#!/usr/bin/env python3
import math
import time
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, String, Float32

# =========================================================
# 너가 채워야 하는 값: 방 20개 "입구/문 앞" 관찰 시작점
# (x, y, yaw_in) : yaw_in은 방 안쪽을 향하게!
# =========================================================
ROOM_ENTRANCES: List[Tuple[float, float, float]] = [
    (0.0, 0.0, 0.0),  # room1 entrance
    (0.0, 0.0, 0.0),  # room2
    (0.0, 0.0, 0.0),  # ...
    (0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0),
]

# Topics (perception node)
TOPIC_LABEL  = "/detections/labels"
TOPIC_DIST   = "/detections/distance"
TOPIC_SPEECH = "/robot_dog/speech"

# Nav timing
STEP_TIMEOUT = 35.0
SETTLE_TIME  = 0.30

# Scan: 360 sweep
SCAN_YAW_OFFSETS = [0.0, math.pi/2, math.pi, -math.pi/2]

# “인식 가능 거리” 반영: 2m 안쪽에서 인식된다면,
# 방 포인트를 안쪽으로 넣어서 dist가 이 근방이 되게 설계
RECOG_DIST_TARGET = 2.0

# 방 안쪽 탐색 포인트: 입구 기준으로 전진 거리 (m)
# 0.8~1.0m는 “문 통과 + 방 안으로 조금” 용도
INROOM_STEPS = [0.8, 1.6, 2.4]  # 필요하면 2개만 써도 됨

# detection 안정화
SAMPLES = 10
SAMPLE_INTERVAL = 0.08
MIN_GOOD_HITS = 2

# dist gate (너무 이상한 값 제외)
DIST_MIN = 0.15
DIST_MAX = 6.0  # 멀리서 안 잡힐 거라 넉넉히

# approach/align by cmd_vel
APPROACH_TIMEOUT = 25.0
FORWARD_SPEED = 0.10
TURN_SPEED = 0.25
STOP_DIST = 1.0  # bark 조건 만들려고 더 가까이

def yaw_to_quaternion(yaw: float):
    return (0.0, 0.0, math.sin(yaw/2.0), math.cos(yaw/2.0))

def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

class Mission2Food(Node):
    def __init__(self):
        super().__init__("mission2_food")

        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.cmd_pub  = self.create_publisher(Twist, "/cmd_vel", 10)

        self.goal_reached = False
        self.goal_sent = False
        self.create_subscription(Bool, "/goal_reached", self.goal_reached_cb, 10)

        self.last_label: str = "None"
        self.last_dist: float = 0.0
        self.last_speech: str = "None"
        self.create_subscription(String, TOPIC_LABEL, self.label_cb, 10)
        self.create_subscription(Float32, TOPIC_DIST, self.dist_cb, 10)
        self.create_subscription(String, TOPIC_SPEECH, self.speech_cb, 10)

        self.started = False
        self.timer = self.create_timer(1.0, self.run_once)

        self.get_logger().info("Mission 2 (2m-recognition) started: enter room -> scan -> approach -> bark -> exit")

    # callbacks
    def goal_reached_cb(self, msg: Bool):
        if msg.data and self.goal_sent:
            self.goal_reached = True

    def label_cb(self, msg: String):
        self.last_label = msg.data.strip()

    def dist_cb(self, msg: Float32):
        self.last_dist = float(msg.data)

    def speech_cb(self, msg: String):
        self.last_speech = msg.data.strip()

    # nav helpers
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

    def wait_goal_reached(self, timeout_s: float) -> bool:
        t0 = time.time()
        while rclpy.ok():
            if self.goal_reached:
                return True
            if time.time() - t0 > timeout_s:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    # detection logic
    def is_good_label(self, label: str) -> bool:
        if not label or label == "None":
            return False
        return "good" in label.lower()

    def good_detected_stably(self) -> bool:
        hits = 0
        for _ in range(SAMPLES):
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.is_good_label(self.last_label) and (DIST_MIN <= self.last_dist <= DIST_MAX):
                hits += 1
            time.sleep(SAMPLE_INTERVAL)
        return hits >= MIN_GOOD_HITS

    def stop_cmd_vel(self):
        self.cmd_pub.publish(Twist())

    # approach/align until perception says bark
    def approach_until_bark(self) -> bool:
        t0 = time.time()
        self.get_logger().info("Approach/Align started (waiting speech='bark').")

        while rclpy.ok():
            if time.time() - t0 > APPROACH_TIMEOUT:
                self.stop_cmd_vel()
                self.get_logger().warn("Approach timeout -> back to exploration.")
                return False

            rclpy.spin_once(self, timeout_sec=0.05)

            if self.last_speech.lower() == "bark":
                self.stop_cmd_vel()
                self.get_logger().info(f"Bark confirmed. label={self.last_label}, dist={self.last_dist:.2f}")
                return True

            dist = self.last_dist
            label = self.last_label.lower()

            tw = Twist()

            # depth 이상이면 회전만
            if dist <= 0.0:
                tw.angular.z = TURN_SPEED
                self.cmd_pub.publish(tw)
                time.sleep(0.12)
                continue

            # good이 보이는 동안만 전진(가까워져서 centered/bark 조건 만들기)
            if "good" in label:
                if dist > STOP_DIST:
                    tw.linear.x = FORWARD_SPEED
                    tw.angular.z = TURN_SPEED * 0.25
                else:
                    tw.linear.x = 0.0
                    tw.angular.z = TURN_SPEED
            else:
                # good을 잃어버리면 회전하면서 다시 찾기
                tw.linear.x = 0.0
                tw.angular.z = TURN_SPEED

            self.cmd_pub.publish(tw)
            time.sleep(0.12)

    # room point generator
    def inroom_points(self, ex: float, ey: float, yaw_in: float) -> List[Tuple[float, float, float]]:
        pts = []
        for d in INROOM_STEPS:
            px = ex + math.cos(yaw_in) * d
            py = ey + math.sin(yaw_in) * d
            pts.append((px, py, yaw_in))
        return pts

    # main
    def run_once(self):
        if self.started:
            return
        self.started = True
        self.timer.cancel()

        for ridx, (ex, ey, yaw_in) in enumerate(ROOM_ENTRANCES, start=1):
            self.get_logger().info(f"[Room {ridx}/20] go entrance viewpoint ({ex:.2f}, {ey:.2f})")
            self.publish_goal(ex, ey, yaw_in)
            if not self.wait_goal_reached(STEP_TIMEOUT):
                self.get_logger().warn(f"[Room {ridx}] entrance timeout -> next")
                continue
            time.sleep(SETTLE_TIME)

            # 방 안쪽으로 2~3개 포인트를 찍으며 스캔
            found_good = False

            for (px, py, pyaw) in self.inroom_points(ex, ey, yaw_in):
                self.get_logger().info(f"[Room {ridx}] go in-room point ({px:.2f}, {py:.2f})")
                self.publish_goal(px, py, pyaw)
                if not self.wait_goal_reached(STEP_TIMEOUT):
                    self.get_logger().warn(f"[Room {ridx}] in-room point timeout -> next point")
                    continue
                time.sleep(SETTLE_TIME)

                # 360 스캔
                for off in SCAN_YAW_OFFSETS:
                    yaw = wrap_pi(pyaw + off)
                    self.publish_goal(px, py, yaw)
                    self.wait_goal_reached(timeout_s=10.0)
                    time.sleep(0.12)

                    if self.good_detected_stably():
                        found_good = True
                        self.get_logger().info(
                            f"[Room {ridx}] GOOD FOUND: label={self.last_label}, dist={self.last_dist:.2f}"
                        )
                        break

                if found_good:
                    break

            if not found_good:
                self.get_logger().info(f"[Room {ridx}] no good food -> next room")
                continue

            # good 발견했으면 접근/정렬해서 bark까지
            if self.approach_until_bark():
                self.get_logger().info("Mission 2 SUCCESS.")
                raise SystemExit

            self.get_logger().info("[Recover] lost target / timeout -> continue exploration")

        self.get_logger().error("Mission 2 ended: failed to find edible food.")
        raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = Mission2Food()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.stop_cmd_vel()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

