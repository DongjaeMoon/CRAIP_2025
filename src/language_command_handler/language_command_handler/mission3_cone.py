#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import sys
import time

# =========================================================
# [설정] 콘 위치 (사용자 환경에 맞게 수정됨)
# =========================================================
CONE_POSES = [
    {'x': 0.25, 'y': 14.6, 'yaw': 1.57},  # 위치 1
    {'x': 1.25, 'y': 14.6, 'yaw': 1.57},  # 위치 2
    {'x': 2.25, 'y': 14.6, 'yaw': 1.57}   # 위치 3
]
# =========================================================

class Mission3Cone(Node):
    def __init__(self):
        super().__init__('mission3_cone')
        
        self.declare_parameter('target_color', 'red')
        self.target_color = self.get_parameter('target_color').get_parameter_value().string_value
        
        self.get_logger().info(f">>> Mission 3 Start! Searching for: {self.target_color.upper()} CONE")

        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.bark_pub = self.create_publisher(String, '/bark', 10)
        self.goal_reached_sub = self.create_subscription(Bool, '/goal_reached', self.goal_reached_callback, 10)
        self.img_sub = self.create_subscription(Image, '/camera_face/image', self.image_callback, 10)
        
        self.bridge = CvBridge()
        self.latest_image = None

        self.current_cone_idx = 0
        self.state = "MOVE" 
        self.waiting_for_arrival = False
        
        # 카메라 안정화 대기용 변수
        self.arrival_time = 0.0 

        self.timer = self.create_timer(0.5, self.control_loop)

    def control_loop(self):
        if self.state == "END":
            # 미션 종료되면 프로세스 죽이기 (터미널에서 확인 가능)
            self.get_logger().info("Mission Finished. Exiting...")
            raise SystemExit

        # 1. 이동 상태
        if self.state == "MOVE":
            if not self.waiting_for_arrival:
                self.send_goal(CONE_POSES[self.current_cone_idx])
                self.waiting_for_arrival = True
        
        # 2. 안정화 대기 (도착 후 1초 기다림 - Sleep 대신 타이머 활용)
        elif self.state == "WAIT_STABLE":
            if time.time() - self.arrival_time > 1.0:
                self.state = "CHECK"

        # 3. 색깔 확인 상태
        elif self.state == "CHECK":
            detected = self.detect_color() # 로그 출력 포함됨
            self.get_logger().info(f"At Cone {self.current_cone_idx+1}: Detected [{detected}] vs Target [{self.target_color}]")
            
            if detected == self.target_color:
                self.get_logger().info(f">>> BINGO! Found {self.target_color.upper()} cone!")
                self.state = "BARK"
            else:
                self.get_logger().info(">>> WRONG COLOR. Planning next move...")
                self.current_cone_idx += 1
                if self.current_cone_idx >= len(CONE_POSES):
                    self.get_logger().error("Checked all cones but target not found.")
                    self.state = "END"
                else:
                    self.state = "MOVE"
                    self.waiting_for_arrival = False

        # 4. 짖기 상태
        elif self.state == "BARK":
            msg = String()
            msg.data = "bark"
            self.bark_pub.publish(msg)
            self.get_logger().info("Bark! Bark!")
            self.state = "END"

    def send_goal(self, pose_data):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = pose_data['x']
        goal.pose.position.y = pose_data['y']
        yaw = pose_data['yaw']
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.goal_pub.publish(goal)
        self.get_logger().info(f"Moving to Cone {self.current_cone_idx+1}...")

    def goal_reached_callback(self, msg):
        # 도착 신호 받으면 대기 모드로 진입 (Sleep 제거)
        if msg.data is True and self.state == "MOVE":
            self.get_logger().info("Arrived. Stabilizing camera...")
            self.arrival_time = time.time()
            self.state = "WAIT_STABLE"

    def image_callback(self, msg):
        self.latest_image = msg

    def detect_color(self):
        if self.latest_image is None:
            self.get_logger().warn("No image received yet!")
            return "none"

        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return "none"

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        h, w, _ = hsv.shape
        center_hsv = hsv[int(h/3):int(2*h/3), int(w/3):int(2*w/3)]

        # HSV 범위 (여기서 튜닝 필요할 수 있음)
        lower_red1 = np.array([0, 100, 100]); upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 100, 100]); upper_red2 = np.array([180, 255, 255])
        lower_green = np.array([40, 100, 100]); upper_green = np.array([80, 255, 255])
        lower_blue = np.array([100, 100, 100]); upper_blue = np.array([140, 255, 255])

        mask_r1 = cv2.inRange(center_hsv, lower_red1, upper_red1)
        mask_r2 = cv2.inRange(center_hsv, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_r1, mask_r2)
        mask_green = cv2.inRange(center_hsv, lower_green, upper_green)
        mask_blue = cv2.inRange(center_hsv, lower_blue, upper_blue)

        red_cnt = cv2.countNonZero(mask_red)
        green_cnt = cv2.countNonZero(mask_green)
        blue_cnt = cv2.countNonZero(mask_blue)

        # [핵심] 디버깅 로그: 어떤 색이 얼마나 잡히는지 확인하세요!
        self.get_logger().info(f"[DEBUG] R:{red_cnt} G:{green_cnt} B:{blue_cnt}")

        max_cnt = max(red_cnt, green_cnt, blue_cnt)
        if max_cnt < 100:
            return "none"
        
        if max_cnt == red_cnt: return "red"
        if max_cnt == green_cnt: return "green"
        if max_cnt == blue_cnt: return "blue"
        
        return "none"

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