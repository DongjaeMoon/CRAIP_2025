#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Bool, Float32MultiArray
from sensor_msgs.msg import LaserScan
import math
import time
import numpy as np
import sys

# =========================================================
# [설정] 파라미터
# =========================================================
# 1. 방 진입 좌표
APPROACH_X = -7.5
APPROACH_Y = -26.0
APPROACH_YAW = -3.0

# 2. 미션 파라미터
TARGET_DIST = 0.85     # 간호사 앞 0.9m 까지 접근
HEX_SIDE_LEN = 0.92    # 육각형 한 변의 길이 (1m)
TARGET_LABEL = "nurse"

# 3. 카메라 파라미터
IMG_WIDTH = 640
FOV_DEG = 80.0        
FOCAL_LENGTH = (IMG_WIDTH / 2) / math.tan(math.radians(FOV_DEG / 2))
# =========================================================

def quaternion_to_yaw(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)

class Mission6Nurse(Node):
    def __init__(self):
        super().__init__('mission6_nurse')
        
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.goal_reached_sub = self.create_subscription(Bool, '/goal_reached', self.goal_reached_cb, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/go1_pose', self.pose_cb, 10)
        
        self.labels_sub = self.create_subscription(String, '/detections/labels', self.labels_cb, 10)
        self.centers_sub = self.create_subscription(Float32MultiArray, '/detections/centers', self.centers_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        # Robot State
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        # Perception Data
        self.found_nurse = False
        self.nurse_cx = -1.0
        self.nurse_idx = -1
        self.last_seen_time = 0.0
        
        # Lidar Data
        self.scan_ranges = []
        self.angle_min = 0.0
        self.angle_inc = 0.0

        # Mission Logic
        self.state = "INIT" 
        self.arrival_wait = False
        self.search_start = 0.0
        
        # Hexagon Variables
        self.hex_edge_count = 0
        self.hex_state = "TURN_OUT"
        self.start_yaw = 0.0
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_check = False
        
        self.timer = self.create_timer(0.05, self.control_loop) # 20Hz
        self.get_logger().info(f">>> Mission 6: Exact Align & REVERSE Hexagon <<<")

    def pose_cb(self, msg: PoseStamped):
        self.robot_x = msg.pose.position.x
        self.robot_y = msg.pose.position.y
        self.robot_yaw = quaternion_to_yaw(msg.pose.orientation)

    def goal_reached_cb(self, msg: Bool):
        if msg.data and self.arrival_wait:
            self.arrival_wait = False

    def scan_cb(self, msg: LaserScan):
        self.scan_ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_inc = msg.angle_increment

    def labels_cb(self, msg: String):
        labels = [l.strip() for l in msg.data.split(',')]
        if TARGET_LABEL in labels:
            try:
                self.nurse_idx = labels.index(TARGET_LABEL)
                self.found_nurse = True
                self.last_seen_time = time.time()
            except:
                self.nurse_idx = -1
                self.found_nurse = False
        else:
            self.found_nurse = False
            
    def centers_cb(self, msg: Float32MultiArray):
        if self.found_nurse and 0 <= self.nurse_idx < len(msg.data):
            self.nurse_cx = msg.data[self.nurse_idx]

    def get_front_lidar_dist(self):
        if not self.scan_ranges: return 99.0
        center_idx = int((0.0 - self.angle_min) / self.angle_inc)
        window = 10 
        dists = []
        for i in range(center_idx - window, center_idx + window + 1):
            if 0 <= i < len(self.scan_ranges):
                r = self.scan_ranges[i]
                if 0.1 < r < 10.0 and not math.isinf(r):
                    dists.append(r)
        return float(np.median(dists)) if dists else 99.0

    def get_angle_error(self):
        pixel_error = (IMG_WIDTH / 2) - self.nurse_cx
        angle = math.atan2(pixel_error, FOCAL_LENGTH)
        return angle

    def stop(self):
        self.cmd_vel_pub.publish(Twist())

    def send_goal(self, x, y, yaw):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        goal.pose.orientation.x = qx
        goal.pose.orientation.y = qy
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw
        self.goal_pub.publish(goal)

    def control_loop(self):
        cmd = Twist()

        # [1] 방으로 이동
        if self.state == "INIT":
            self.get_logger().info(f">>> [INIT] Going to Break Room...")
            self.send_goal(APPROACH_X, APPROACH_Y, APPROACH_YAW)
            self.state = "APPROACH"
            self.arrival_wait = True

        # [2] 도착 -> 탐색 시작
        elif self.state == "APPROACH":
            if not self.arrival_wait:
                self.get_logger().info(">>> [APPROACH] Arrived. Searching for Nurse...")
                self.state = "SEARCH"
                self.search_start = time.time()

        # [3] 탐색 (Search)
        elif self.state == "SEARCH":
            if self.found_nurse and self.nurse_cx > 0:
                self.stop()
                self.get_logger().info(f">>> [SEARCH] Found Nurse! (cx={self.nurse_cx:.1f}). Aligning...")
                self.state = "ALIGN"
                time.sleep(0.5)
                return

            if time.time() - self.search_start < 30.0:
                cmd.angular.z = 0.3 
                self.cmd_vel_pub.publish(cmd)
            else:
                self.stop()
                self.get_logger().error(">>> [FAIL] Could not find nurse. Stopping.")
                self.state = "FAIL"

        # [4] 정밀 정렬
        elif self.state == "ALIGN":
            if not self.found_nurse:
                self.get_logger().warn(">>> [ALIGN] Lost target. Re-searching...")
                self.state = "SEARCH"
                self.search_start = time.time()
                return

            angle_err = self.get_angle_error() 
            
            if abs(angle_err) < 0.035:
                self.stop()
                self.get_logger().info(">>> [ALIGN] Perfect Alignment! Moving Closer...")
                self.state = "MOVE_TO"
                time.sleep(0.5)
            else:
                cmd.angular.z = 1.5 * angle_err
                cmd.angular.z = max(min(cmd.angular.z, 0.5), -0.5)
                self.cmd_vel_pub.publish(cmd)

        # [5] 접근 (Approach)
        elif self.state == "MOVE_TO":
            dist = self.get_front_lidar_dist()
            
            if abs(dist - TARGET_DIST) < 0.1:
                self.stop()
                self.get_logger().info(f">>> [MOVE_TO] Reached Target ({dist:.2f}m). Starting Hexagon!")
                self.state = "HEX_START"
            elif dist < TARGET_DIST:
                cmd.linear.x = -0.15
                self.cmd_vel_pub.publish(cmd)
            else:
                cmd.linear.x = 0.25
                if self.found_nurse:
                    angle_err = self.get_angle_error()
                    cmd.angular.z = 1.5 * angle_err
                self.cmd_vel_pub.publish(cmd)

        # [6] 육각형 그리기 (사용자 요청 로직 적용)
        elif self.state == "HEX_START":
            self.hex_edge_count = 0
            self.hex_state = "TURN_OUT" # 첫 턴은 왼쪽
            self.start_check = False
            self.state = "HEX_RUN"
            self.get_logger().info(">>> [HEX] Pattern: First Left -> Then Rights")

        elif self.state == "HEX_RUN":
            if self.hex_edge_count >= 6:
                self.stop()
                self.get_logger().info(">>> [END] Complete!")
                self.state = "END"
                return

            # --- 1. 초기 회전: 좌회전 (TURN_OUT) ---
            if self.hex_state == "TURN_OUT":
                if not self.start_check:
                    self.start_yaw = self.robot_yaw
                    self.start_check = True
                
                diff = self.robot_yaw - self.start_yaw
                while diff > math.pi: diff -= 2*math.pi
                while diff < -math.pi: diff += 2*math.pi
                
                # 좌회전 60도 (+0.4)
                if abs(diff) >= math.radians(60):
                    self.stop()
                    self.hex_state = "MOVE_STRAIGHT"
                    self.start_check = False
                    self.get_logger().info(">>> [HEX] Initial Left Turn Done.")
                else:
                    cmd.angular.z = 0.4 # [좌회전]
                    self.cmd_vel_pub.publish(cmd)

            # --- 2. 직진 (변 그리기) ---
            elif self.hex_state == "MOVE_STRAIGHT":
                if not self.start_check:
                    self.start_x = self.robot_x
                    self.start_y = self.robot_y
                    self.start_check = True
                
                dist_moved = math.hypot(self.robot_x - self.start_x, self.robot_y - self.start_y)
                
                if dist_moved >= HEX_SIDE_LEN:
                    self.stop()
                    self.hex_state = "TURN_CORNER"
                    self.start_check = False
                    self.hex_edge_count += 1
                    self.get_logger().info(f"    - Edge {self.hex_edge_count} Done.")
                else:
                    cmd.linear.x = 0.25
                    self.cmd_vel_pub.publish(cmd)

            # --- 3. 코너 회전: 우회전 (TURN_CORNER) ---
            elif self.hex_state == "TURN_CORNER":
                if not self.start_check:
                    self.start_yaw = self.robot_yaw
                    self.start_check = True
                
                diff = self.robot_yaw - self.start_yaw
                while diff > math.pi: diff -= 2*math.pi
                while diff < -math.pi: diff += 2*math.pi
                
                # 우회전 60도 (-0.4)
                if abs(diff) >= math.radians(58):
                    self.stop()
                    self.hex_state = "MOVE_STRAIGHT"
                    self.start_check = False
                else:
                    cmd.angular.z = -0.4 # [우회전]
                    self.cmd_vel_pub.publish(cmd)

        elif self.state == "FAIL" or self.state == "END":
            self.stop()
            # [수정] 노드 종료 시 로그 출력 후 루프 탈출
            if self.state == "END":
                self.get_logger().info(">>> Mission 6 Finished Successfully. Shutting down node.")
            else:
                self.get_logger().error(">>> Mission 6 Failed. Shutting down node.")
            
            # 타이머 멈춤 (더 이상 콜백 실행 안 함)
            self.timer.cancel()
            # 1초 뒤에 프로세스 종료 (로그 출력 시간 확보)
            time.sleep(1.0)
            raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = Mission6Nurse()
    try:
        rclpy.spin(node)
    except SystemExit:
        # [중요] SystemExit을 잡아서 정상 종료 처리
        rclpy.logging.get_logger("mission6_nurse").info("Node stopped cleanly.")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # rclpy.shutdown()은 한 번만 호출
        if rclpy.ok():
            rclpy.shutdown()
        #rclpy.shutdown()

if __name__ == '__main__':
    main()