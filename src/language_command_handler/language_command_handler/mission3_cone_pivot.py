#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Bool
import math
import time

# =========================================================
# [설정 1] 관측 베이스캠프 위치 (중앙 지점)
# 콘(y=14.85)보다 약 1.3m 앞선 지점
# =========================================================
OBS_X = 1.25   
OBS_Y = 15.3   # [수정됨] 15.3은 콘 뒤쪽이라 13.5(앞쪽)로 변경 권장

# =========================================================
# [설정 2] 회전 각도 (Left -> Right -> Center)
# 소거법 적용: 양 끝을 먼저 보고, 없으면 가운데로 확정
# =========================================================
SCAN_YAWS = [
    2.1,   # [1번] 왼쪽 (약 115도)
    0.8,   # [2번] 오른쪽 (약 45도)
    1.57   # [3번] 중앙 (90도)
]

# =========================================================
# [설정 3] 정답 발견 시 앞으로 다가갈 거리 및 속도
# =========================================================
APPROACH_DISTANCE = 0.5  # 앞으로 갈 거리 (m)
FORWARD_SPEED = 0.1      # 전진 속도 (m/s)

# Perception
TOPIC_LABELS = "/detections/labels"
CHECK_DURATION = 5.0  
MIN_HITS = 4          

class Mission3ConePivot(Node):
    def __init__(self):
        super().__init__('mission3_cone_pivot')
        
        self.declare_parameter('target_color', 'red')
        self.target_color = self.get_parameter('target_color').get_parameter_value().string_value.lower()
        
        self.get_logger().info(f">>> [Pivot Strategy] Target: {self.target_color.upper()} CONE")
        self.get_logger().info(f"Basecamp: ({OBS_X}, {OBS_Y})")

        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.bark_pub = self.create_publisher(String, '/bark', 10)
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10) # [추가] 직접 제어용
        
        # Subscribers
        self.goal_reached_sub = self.create_subscription(
            Bool, '/goal_reached', self.goal_reached_callback, 10
        )
        self.labels_sub = self.create_subscription(
            String, TOPIC_LABELS, self.labels_callback, 10
        )

        # State Variables
        self.scan_idx = 0
        self.state = "INIT"
        self.waiting_for_arrival = False
        self.last_labels = []
        self.color_hits = 0
        self.check_start_time = 0.0
        
        # 접근 제어용
        self.is_approaching = False
        self.approach_start_time = 0.0
        
        self.timer = self.create_timer(0.2, self.control_loop)

    def labels_callback(self, msg: String):
        raw = msg.data.strip()
        if not raw or raw == "None":
            self.last_labels = []
        else:
            self.last_labels = [label.strip().lower() for label in raw.split(',')]

    def goal_reached_callback(self, msg: Bool):
        # 네비게이션(이동/회전) 완료 신호 처리
        if msg.data is True:
            if self.state == "INIT":
                self.get_logger().info("Arrived at Basecamp.")
                self.state = "ROTATE"
                self.waiting_for_arrival = False
                
            elif self.state == "ROTATE":
                self.get_logger().info(f"Aligned to Angle {self.scan_idx + 1}.")
                self.state = "CHECK"
                self.check_start_time = time.time()
                self.color_hits = 0
                self.waiting_for_arrival = False

    def control_loop(self):
        if self.state == "END":
            raise SystemExit

        # [Step 1] 베이스캠프로 이동 (Navigation)
        if self.state == "INIT":
            if not self.waiting_for_arrival:
                self.get_logger().info(f"Moving to Basecamp ({OBS_X}, {OBS_Y})...")
                self.send_goal(OBS_X, OBS_Y, 1.57) # 처음엔 정면 보기
                self.waiting_for_arrival = True

        # [Step 2] 다음 각도로 제자리 회전 (Navigation)
        elif self.state == "ROTATE":
            if not self.waiting_for_arrival:
                if self.scan_idx >= len(SCAN_YAWS):
                    self.get_logger().error("Target not found in all angles.")
                    self.state = "END"
                    return

                target_yaw = SCAN_YAWS[self.scan_idx]
                self.get_logger().info(f"Rotating to angle {self.scan_idx+1} ({target_yaw} rad)...")
                
                # 위치는 고정, 각도만 변경
                self.send_goal(OBS_X, OBS_Y, target_yaw)
                self.waiting_for_arrival = True

        # [Step 3] 색상 확인
        elif self.state == "CHECK":
            found_now = False
            for label in self.last_labels:
                if self.target_color in label:
                    found_now = True
                    break
            
            if found_now:
                self.color_hits += 1
            
            # 일정 시간 관측 후 판단
            if time.time() - self.check_start_time > CHECK_DURATION:
                if self.color_hits >= MIN_HITS:
                    self.get_logger().info(f">>> FOUND TARGET: {self.target_color.upper()}!")
                    self.state = "APPROACH"
                    self.is_approaching = False 
                else:
                    self.get_logger().info("Not matched. Checking next angle...")
                    self.scan_idx += 1
                    self.state = "ROTATE"
                    self.waiting_for_arrival = False

        # [Step 4] 정답을 향해 직진 (Direct Control)
        # 경로 생성 없이 시간(Time) 기반으로 앞으로 전진
        elif self.state == "APPROACH":
            if not self.is_approaching:
                self.get_logger().info(f"Approaching Target Blindly ({APPROACH_DISTANCE}m)...")
                self.approach_start_time = time.time()
                self.is_approaching = True
            
            # 이동 시간 = 거리 / 속도
            required_time = APPROACH_DISTANCE / FORWARD_SPEED
            
            if time.time() - self.approach_start_time < required_time:
                # 계속 전진 명령 발행
                cmd = Twist()
                cmd.linear.x = FORWARD_SPEED
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)
            else:
                # 시간 종료 -> 정지
                self.stop_robot()
                self.get_logger().info("Approach Finished.")
                self.state = "BARK"

        # [Step 5] 짖기
        elif self.state == "BARK":
            msg = String()
            msg.data = "bark"
            self.bark_pub.publish(msg)
            self.get_logger().info("Bark! Bark!")
            self.state = "END"

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
    node = Mission3ConePivot()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()