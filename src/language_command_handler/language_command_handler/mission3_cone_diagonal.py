#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Bool, Float32MultiArray
import math
import time

# =========================================================
# [설정 1] 사선 관측 위치 (Diagonal Observation Pose)
# =========================================================
OBS_X = 2.5
OBS_Y = 13.5
OBS_YAW = 1.8

# =========================================================
# [설정 2] 콘들의 실제 위치 (왼쪽 -> 오른쪽 순서)
# =========================================================
CONE_LOCATIONS = [
    {'x': 0.1,  'y': 15.25},  # [0] left
    {'x': 1.25, 'y': 15.25},  # [1] center
    {'x': 2.5,  'y': 15.25}   # [2] right
]

# =========================================================
# [설정 3] 접근 파라미터
# =========================================================
NAV_STOP_OFFSET = 0.5
APPROACH_DRIVE_DIST = 1.15
FORWARD_SPEED = 0.2

# Perception Topics
TOPIC_LABELS = "/detections/labels"
TOPIC_DISTS  = "/detections/distances"

# Constants
CHECK_DURATION = 5.0

# Bark 유지
TOPIC_SPEECH = "/robot_dog/speech"
BARK_PERIOD = 0.001  # 0 금지, 0.1 추천

class Mission3ConeDiagonal(Node):
    def __init__(self):
        super().__init__('mission3_cone_diagonal')

        self.declare_parameter('target_color', 'red')
        self.target_color = self.get_parameter('target_color').get_parameter_value().string_value.lower()
        self.get_logger().info(f">>> [Diagonal Strategy] Target: {self.target_color.upper()}")

        # Publishers
        self.goal_pub   = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.bark_pub   = self.create_publisher(String, '/bark', 10)          # 기존 유지
        self.speech_pub = self.create_publisher(String, TOPIC_SPEECH, 10)     # 유지용
        self.cmd_pub    = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.goal_reached_sub = self.create_subscription(
            Bool, '/goal_reached', self.goal_reached_callback, 10
        )
        self.labels_sub = self.create_subscription(
            String, TOPIC_LABELS, self.labels_callback, 10
        )
        self.dists_sub = self.create_subscription(
            Float32MultiArray, TOPIC_DISTS, self.dists_callback, 10
        )

        # State
        # INIT -> SCAN -> NAV_TO_CONE -> BLIND_APPROACH -> BARK(유지)
        self.state = "INIT"
        self.waiting_for_arrival = False

        self.current_labels = []
        self.current_dists = []

        self.scan_start_time = 0.0
        self.detected_cones = []   # [(dist, label), ...]
        self.target_cone_idx = -1

        self.approach_start_time = 0.0
        self.is_approaching = False

        # bark 유지 모드
        self.barking = False
        self.bark_timer = None

        self.timer = self.create_timer(0.2, self.control_loop)

    # -------------------------
    # Callbacks
    # -------------------------
    def labels_callback(self, msg: String):
        raw = msg.data.strip()
        if not raw or raw == "None":
            self.current_labels = []
        else:
            self.current_labels = [label.strip().lower() for label in raw.split(',')]

    def dists_callback(self, msg: Float32MultiArray):
        self.current_dists = list(msg.data)

    def goal_reached_callback(self, msg: Bool):
        if msg.data is True:
            if self.state == "INIT":
                self.get_logger().info("Arrived at Observation Point.")
                self.state = "SCAN"
                self.scan_start_time = time.time()
                self.waiting_for_arrival = False

            elif self.state == "NAV_TO_CONE":
                self.get_logger().info("Arrived in front of target cone.")
                self.state = "BLIND_APPROACH"
                self.waiting_for_arrival = False

    # -------------------------
    # Bark 유지 helpers
    # -------------------------
    def _publish_bark_forever(self):
        m = String()
        m.data = "bark"
        self.speech_pub.publish(m)

    def _start_bark_forever(self):
        if self.barking:
            return
        self.barking = True

        # 즉시 1번
        self._publish_bark_forever()

        # 계속 유지
        self.bark_timer = self.create_timer(BARK_PERIOD, self._publish_bark_forever)

    # -------------------------
    # Main Loop
    # -------------------------
    def control_loop(self):
        # bark 모드면 다른 로직 안 하고 계속 유지
        if self.barking:
            return

        # [Step 1] 관측 위치로 이동
        if self.state == "INIT":
            if not self.waiting_for_arrival:
                self.get_logger().info(f"Moving to Diagonal Spot ({OBS_X}, {OBS_Y})...")
                self.send_goal(OBS_X, OBS_Y, OBS_YAW)
                self.waiting_for_arrival = True

        # [Step 2] 스캔 + 정렬
        elif self.state == "SCAN":
            if len(self.current_labels) == len(self.current_dists) and len(self.current_labels) > 0:
                temp_cones = []
                for i in range(len(self.current_labels)):
                    lbl = self.current_labels[i]
                    dst = self.current_dists[i]
                    if "cone" in lbl:
                        temp_cones.append((dst, lbl))

                temp_cones.sort(key=lambda x: x[0], reverse=True)  # 멀 -> 가
                self.detected_cones = temp_cones

            if time.time() - self.scan_start_time > CHECK_DURATION:
                if len(self.detected_cones) < 2:
                    self.get_logger().warn("Not enough cones detected. Rescan...")
                    self.scan_start_time = time.time()
                    return

                found_idx = -1
                for idx, (dst, lbl) in enumerate(self.detected_cones):
                    if self.target_color in lbl:
                        found_idx = idx
                        break

                if found_idx != -1:
                    if found_idx > 2:
                        found_idx = 2
                    self.target_cone_idx = found_idx
                    self.get_logger().info(f">>> ANALYSIS COMPLETE: idx={found_idx} (0:L,1:C,2:R)")
                    self.state = "NAV_TO_CONE"
                else:
                    self.get_logger().error("Target not seen. Rescan...")
                    self.scan_start_time = time.time()

        # [Step 3] 해당 콘 앞으로 이동
        elif self.state == "NAV_TO_CONE":
            if not self.waiting_for_arrival:
                target_loc = CONE_LOCATIONS[self.target_cone_idx]
                goal_x = target_loc['x']
                goal_y = target_loc['y'] - NAV_STOP_OFFSET
                self.get_logger().info(f"Navigating to Pre-Action Pose: ({goal_x}, {goal_y})")
                self.send_goal(goal_x, goal_y, 1.5)
                self.waiting_for_arrival = True

        # [Step 4] 맹목적 전진 -> 끝나면 즉시 bark 시작
        elif self.state == "BLIND_APPROACH":
            if not self.is_approaching:
                self.get_logger().info("Blind Approach Started...")
                self.approach_start_time = time.time()
                self.is_approaching = True

            required_time = APPROACH_DRIVE_DIST / FORWARD_SPEED

            if time.time() - self.approach_start_time < required_time:
                cmd = Twist()
                cmd.linear.x = FORWARD_SPEED
                self.cmd_pub.publish(cmd)
            else:
                self.stop_robot()
                self.is_approaching = False
                self.get_logger().info("Blind approach finished -> START BARK now")
                self.state = "BARK"

        # [Step 5] bark 발행 + 유지 (여기서 끝내지 말고 계속 유지)
        elif self.state == "BARK":
            # /bark 1회
            msg = String()
            msg.data = "bark"
            self.bark_pub.publish(msg)

            # /robot_dog/speech 계속 유지
            self._start_bark_forever()

            self.get_logger().info("BARK started and will be kept (node stays alive)")

    # -------------------------
    # Helpers
    # -------------------------
    def send_goal(self, x, y, yaw):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_pub.publish(goal)

    def stop_robot(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = Mission3ConeDiagonal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
