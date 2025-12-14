#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
import math
import sys

# =========================================================
# [설정] 화장실 위치 좌표 (Rviz에서 확인한 값으로 수정 필수)
# =========================================================
TOILET_X = -7.2      # 예시 (반드시 수정하세요)
TOILET_Y = -2.4       # 예시
TOILET_YAW = 1.57    # 90도 (라디안)
# =========================================================

def yaw_to_quaternion(yaw):
    """Yaw 각도를 ROS Quaternion으로 변환"""
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return qx, qy, qz, qw

class Mission1Toilet(Node):
    def __init__(self):
        super().__init__('mission1_toilet')

        # 1. Publisher & Subscriber 설정
        # A* Planner에게 목표 전달
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        # 미션 완료 후 짖기
        self.bark_pub = self.create_publisher(String, '/bark', 10)
        
        # Path Tracker가 보내주는 "도착했다" 신호 구독
        self.goal_reached_sub = self.create_subscription(
            Bool,
            '/goal_reached',
            self.goal_reached_callback,
            10
        )
        
        # 2. 노드 실행 1초 뒤에 목표지점 전송 (안정화 대기)
        self.timer = self.create_timer(1.0, self.send_goal_once)
        self.goal_sent = False
        
        self.get_logger().info("Mission 1 Initiated: Going to Toilet...")

    def send_goal_once(self):
        if self.goal_sent:
            return

        # 목표 Pose 메시지 생성
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        
        goal.pose.position.x = TOILET_X
        goal.pose.position.y = TOILET_Y
        
        qx, qy, qz, qw = yaw_to_quaternion(TOILET_YAW)
        goal.pose.orientation.x = qx
        goal.pose.orientation.y = qy
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw
        
        # 발행
        self.goal_pub.publish(goal)
        self.get_logger().info(f"Published Goal: x={TOILET_X}, y={TOILET_Y}")
        
        self.goal_sent = True
        self.timer.cancel() # 타이머 종료

    def goal_reached_callback(self, msg):
        # Path Tracker가 True를 보내면 도착 완료
        if msg.data is True and self.goal_sent:
            self.get_logger().info(">>> Goal Reached Signal Received!")
            
            # 멍멍!
            bark_msg = String()
            bark_msg.data = "bark"
            self.bark_pub.publish(bark_msg)
            self.get_logger().info("Bark! Mission Complete.")
            
            # 미션 종료 (노드 죽이기)
            # LLM 핸들러는 프로세스 종료를 감지하고 다음 명령을 대기 상태로 전환함
            raise SystemExit 

def main(args=None):
    rclpy.init(args=args)
    node = Mission1Toilet()
    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.logging.get_logger("mission1_toilet").info("Mission Node Exiting...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
