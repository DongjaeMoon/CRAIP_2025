#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool, Float32MultiArray
import math
import time

# =========================================================
# [설정] 콘 탐색 위치 후보 (Map Frame 좌표)
# =========================================================
CONE_POSES = [
    {'x': 0.1, 'y': 14.85, 'yaw': 1.57},  # 위치 1 (왼쪽)
    {'x': 1.25, 'y': 14.85, 'yaw': 1.57},  # 위치 2 (중간)
    {'x': 2.5, 'y': 14.85, 'yaw': 1.57}   # 위치 3 (오른쪽)
]
# =========================================================

# Perception Topics
TOPIC_LABELS = "/detections/labels"       # String CSV: "box, red_cone, ..."
TOPIC_DISTS  = "/detections/distances"    # Float32MultiArray

# Detection Constants
CHECK_DURATION = 2.5      # 도착 후 관측 시간
MIN_HITS = 3              # 관측 시간 동안 최소 몇 번 이상 감지되어야 하는지

class Mission3Cone(Node):
    def __init__(self):
        super().__init__('mission3_cone')
        
        # 1. 파라미터 받기 (launch 파일에서 넘어옴: red, blue, green)
        self.declare_parameter('target_color', 'red')
        self.target_color = self.get_parameter('target_color').get_parameter_value().string_value.lower()
        
        self.get_logger().info(f">>> Mission 3 Start! Target: {self.target_color.upper()} CONE")

        # 2. Publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.bark_pub = self.create_publisher(String, '/bark', 10)

        # 3. Subscribers
        self.goal_reached_sub = self.create_subscription(
            Bool, '/goal_reached', self.goal_reached_callback, 10
        )
        
        # Perception Node 구독 (이미지 대신 라벨/거리 사용)
        self.labels_sub = self.create_subscription(
            String, TOPIC_LABELS, self.labels_callback, 10
        )
        self.dists_sub = self.create_subscription(
            Float32MultiArray, TOPIC_DISTS, self.dists_callback, 10
        )

        # 4. State Variables
        self.current_cone_idx = 0
        self.state = "MOVE"  # MOVE -> WAIT -> CHECK -> NEXT or BARK
        self.waiting_for_arrival = False
        
        self.last_labels = []
        self.last_dists = []
        
        # 색상 확인용 카운터
        self.color_hits = 0
        self.check_start_time = 0.0

        # 메인 루프 (0.5초 주기)
        self.timer = self.create_timer(0.5, self.control_loop)

    # ---------------------------------------------------------
    # Callbacks
    # ---------------------------------------------------------
    def labels_callback(self, msg: String):
        # CSV 문자열 파싱 (예: "person, red_cone")
        raw = msg.data.strip()
        if not raw or raw == "None":
            self.last_labels = []
        else:
            self.last_labels = [label.strip().lower() for label in raw.split(',')]

    def dists_callback(self, msg: Float32MultiArray):
        self.last_dists = list(msg.data)

    def goal_reached_callback(self, msg: Bool):
        if msg.data is True and self.state == "MOVE":
            self.get_logger().info("Arrived at candidate pose.")
            self.state = "WAIT"

    # ---------------------------------------------------------
    # Main Logic
    # ---------------------------------------------------------
    def control_loop(self):
        if self.state == "END":
            # 프로세스 종료
            raise SystemExit

        # [1] 이동 명령 발행
        if self.state == "MOVE":
            if not self.waiting_for_arrival:
                self.send_goal(CONE_POSES[self.current_cone_idx])
                self.waiting_for_arrival = True
        
        # [2] 도착 후 안정화 (잠시 대기)
        elif self.state == "WAIT":
            self.get_logger().info("Stabilizing and Checking Perception...")
            self.check_start_time = time.time()
            self.color_hits = 0
            self.state = "CHECK"

        # [3] 인식 확인 (일정 시간 동안 누적 확인)
        elif self.state == "CHECK":
            # 현재 보이는 라벨 중에 타겟 색상이 포함된 라벨이 있는지 확인
            # 예: target="red", labels=["red_cone", "box"] -> Found!
            found_now = False
            for label in self.last_labels:
                if self.target_color in label: # "red" 문자열이 "red_cone"에 포함됨
                    found_now = True
                    break
            
            if found_now:
                self.color_hits += 1
                self.get_logger().info(f"Found target '{self.target_color}' in labels! (Hits: {self.color_hits})")

            # 일정 시간(CHECK_DURATION)이 지났는지 확인
            if time.time() - self.check_start_time > CHECK_DURATION:
                if self.color_hits >= MIN_HITS:
                    self.get_logger().info(f">>> BINGO! Found {self.target_color.upper()} cone.")
                    self.state = "BARK"
                else:
                    self.get_logger().info(f">>> Target {self.target_color} NOT found here. Moving to next...")
                    self.next_cone()

        # [4] 정답을 찾았을 때 -> 짖기
        elif self.state == "BARK":
            msg = String()
            msg.data = "bark"
            self.bark_pub.publish(msg)
            self.get_logger().info("Bark! Bark! Mission Complete.")
            self.state = "END"

    def next_cone(self):
        self.current_cone_idx += 1
        if self.current_cone_idx >= len(CONE_POSES):
            self.get_logger().error("Checked all locations but target NOT found.")
            self.state = "END"
        else:
            self.state = "MOVE"
            self.waiting_for_arrival = False

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
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()