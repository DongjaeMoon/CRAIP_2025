#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Bool, Float32MultiArray
import math
import time

# =========================================================
# [설정] 콘 탐색 위치 후보 (Map Frame 좌표)
# =========================================================
CONE_POSES = [
    {'x': 0.4, 'y': 14.6, 'yaw': 1.7},  # 위치 1 (왼쪽)
    {'x': 2.2, 'y': 14.6, 'yaw': 1.2},  # 위치 2 (중간)
    {'x': 1.25, 'y': 15.3, 'yaw': 1.6}  # 위치 3 (오른쪽)
]
# =========================================================

# Perception Topics (원래 코드 유지용)
TOPIC_LABELS = "/detections/labels"
TOPIC_DISTS  = "/detections/distances"

# Nav/pose
TOPIC_GOAL = "/goal_pose"
TOPIC_LOCAL_POSE = "/go1_pose"
TOPIC_GOAL_REACHED = "/goal_reached"

# ✅ 속도 구독 (멈춤 판정용)
TOPIC_CMD = "/cmd_vel"

# Bark 유지
TOPIC_SPEECH = "/robot_dog/speech"
BARK_PERIOD = 0.1  # 10Hz 권장

# ✅ "근처 도착" 판정 반경
GOAL_RADIUS = 2.0

# ✅ "완전히 멈춤" 판정 기준
STOP_LIN_EPS = 0.03   # m/s
STOP_ANG_EPS = 0.05   # rad/s
STOP_HOLD_TIME = 0.3  # seconds: 이 시간 동안 멈춤이 유지되면 bark 시작

def dist2d(ax, ay, bx, by):
    return math.hypot(ax - bx, ay - by)

class Mission3Cone(Node):
    def __init__(self):
        super().__init__('mission3_cone')

        self.declare_parameter('target_color', 'red')
        self.target_color = self.get_parameter('target_color').get_parameter_value().string_value.lower()
        self.get_logger().info(f">>> Mission 3 Start! Target: {self.target_color.upper()} CONE")

        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, TOPIC_GOAL, 10)
        self.speech_pub = self.create_publisher(String, TOPIC_SPEECH, 10)

        # Subscribers
        self.goal_reached_sub = self.create_subscription(Bool, TOPIC_GOAL_REACHED, self.goal_reached_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, TOPIC_LOCAL_POSE, self.pose_callback, 10)
        self.cmd_sub  = self.create_subscription(Twist, TOPIC_CMD, self.cmd_callback, 10)

        # (원래 코드 유지용 구독)
        self.labels_sub = self.create_subscription(String, TOPIC_LABELS, self.labels_callback, 10)
        self.dists_sub = self.create_subscription(Float32MultiArray, TOPIC_DISTS, self.dists_callback, 10)

        # State
        self.current_cone_idx = 0
        self.state = "MOVE"   # MOVE -> NEAR(멈춤대기) -> BARK
        self.waiting_for_arrival = False

        # pose / goal
        self.cur_x = None
        self.cur_y = None
        self.target_x = None
        self.target_y = None

        # cmd_vel (멈춤 판정)
        self.last_lin = 0.0
        self.last_ang = 0.0
        self.stop_since = None  # 멈춤 시작 시간(연속 유지 체크)

        # (원래 코드 유지용)
        self.last_labels = []
        self.last_dists = []

        # bark 유지 모드
        self.barking = False
        self.bark_timer = None

        # main loop
        self.timer = self.create_timer(0.1, self.control_loop)

    # -------------------------
    # Callbacks
    # -------------------------
    def pose_callback(self, msg: PoseStamped):
        self.cur_x = msg.pose.position.x
        self.cur_y = msg.pose.position.y

    def cmd_callback(self, msg: Twist):
        self.last_lin = float(msg.linear.x)
        self.last_ang = float(msg.angular.z)

    def labels_callback(self, msg: String):
        raw = msg.data.strip()
        if not raw or raw == "None":
            self.last_labels = []
        else:
            self.last_labels = [label.strip().lower() for label in raw.split(',')]

    def dists_callback(self, msg: Float32MultiArray):
        self.last_dists = list(msg.data)

    def goal_reached_callback(self, msg: Bool):
        # goal_reached도 참고만: "near"로 넘어가서 멈춤 확인 후 bark
        if self.barking:
            return
        if msg.data and self.state == "MOVE":
            self.get_logger().info("goal_reached=True -> enter NEAR (wait stop)")
            self.state = "NEAR"
            self.stop_since = None

    # -------------------------
    # Bark 유지
    # -------------------------
    def _publish_bark(self):
        m = String()
        m.data = "bark"
        self.speech_pub.publish(m)

    def _start_bark_forever(self):
        if self.barking:
            return
        self.barking = True
        self.get_logger().info("BARK started -> keep barking forever")
        self._publish_bark()
        self.bark_timer = self.create_timer(BARK_PERIOD, self._publish_bark)

    # -------------------------
    # Control loop
    # -------------------------
    def control_loop(self):
        if self.barking:
            return

        if self.state == "MOVE":
            # goal 1회 발행
            if not self.waiting_for_arrival:
                pose = CONE_POSES[self.current_cone_idx]
                self.send_goal(pose)
                self.waiting_for_arrival = True
                self.target_x = float(pose["x"])
                self.target_y = float(pose["y"])
                self.stop_since = None

            # goal 근처 들어오면 NEAR로 (멈춤 확인 단계)
            if (self.cur_x is not None) and (self.target_x is not None):
                d = dist2d(self.cur_x, self.cur_y, self.target_x, self.target_y)
                if d <= GOAL_RADIUS:
                    self.get_logger().info(f"Near goal (d={d:.2f} <= {GOAL_RADIUS}) -> enter NEAR (wait stop)")
                    self.state = "NEAR"
                    self.stop_since = None

        elif self.state == "NEAR":
            # "완전 정지" 판정: cmd_vel이 거의 0인 상태가 STOP_HOLD_TIME 연속 유지
            stopped_now = (abs(self.last_lin) <= STOP_LIN_EPS) and (abs(self.last_ang) <= STOP_ANG_EPS)

            if stopped_now:
                if self.stop_since is None:
                    self.stop_since = time.time()
                elif (time.time() - self.stop_since) >= STOP_HOLD_TIME:
                    self.get_logger().info("Robot fully stopped -> START BARK")
                    self.state = "BARK"
            else:
                # 다시 움직이면 타이머 리셋
                self.stop_since = None

        elif self.state == "BARK":
            self._start_bark_forever()

    # -------------------------
    # Helpers
    # -------------------------
    def send_goal(self, pose_data):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(pose_data['x'])
        goal.pose.position.y = float(pose_data['y'])

        yaw = float(pose_data['yaw'])
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)

        self.goal_pub.publish(goal)
        self.get_logger().info(f"Moving to location {self.current_cone_idx+1}...")

def main(args=None):
    rclpy.init(args=args)
    node = Mission3Cone()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
