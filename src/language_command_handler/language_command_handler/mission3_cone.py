#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Bool, Float32MultiArray
import math
import time
import sys # [추가] 종료를 위해 필요

# =========================================================
# [설정] 파라미터
# =========================================================
OBS_X = 1.25
OBS_Y = 13.25
OBS_YAW = 1.57  # 정면(90도)을 바라보고 섭니다.

# 콘 Y 좌표 (고정)
CONE_Y = 15.25

# 접근 설정
STOP_DIST = 0.5       # 콘 앞 0.5m 에서 정지
FORWARD_SPEED = 0.2   # 전진 속도 (m/s)

IMG_WIDTH = 640
FOV_DEG = 80.0        
FOCAL_LENGTH = (IMG_WIDTH / 2) / math.tan(math.radians(FOV_DEG / 2))
# =========================================================

def quaternion_to_yaw(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class Mission3ConeDiagonal(Node):
    def __init__(self):
        super().__init__('mission3_cone_diagonal')

        self.declare_parameter('target_color', 'red')
        self.target_color = self.get_parameter('target_color').get_parameter_value().string_value.lower()
        self.get_logger().info(f">>> [System] Target: {self.target_color.upper()}")

        # Publishers
        self.goal_pub   = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.bark_pub   = self.create_publisher(String, '/bark', 10)
        self.speech_pub = self.create_publisher(String, '/robot_dog/speech', 10)
        self.cmd_pub    = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.goal_reached_sub = self.create_subscription(Bool, '/goal_reached', self.goal_reached_cb, 10)
        self.labels_sub = self.create_subscription(String, '/detections/labels', self.labels_cb, 10)
        self.centers_sub = self.create_subscription(Float32MultiArray, '/detections/centers', self.centers_cb, 10)
        self.dists_sub = self.create_subscription(Float32MultiArray, '/detections/distances', self.dists_cb, 10)

        # Variables
        self.state = "INIT" 
        self.arrival_wait = False
        
        self.found_target = False
        self.target_cx = -1.0
        self.target_idx = -1
        
        # Timing Variables
        self.observe_start_time = 0.0
        self.approach_start_time = 0.0
        self.approach_duration = 0.0

        # [수정 3] Bark 난사용 타이머 변수
        self.aggressive_bark_timer = None
        
        self.timer = self.create_timer(0.1, self.control_loop)

    def goal_reached_cb(self, msg: Bool):
        if msg.data and self.arrival_wait:
            self.arrival_wait = False

    def labels_cb(self, msg: String):
        labels = [l.strip() for l in msg.data.split(',')]
        found = False
        for i, lbl in enumerate(labels):
            if self.target_color in lbl and "cone" in lbl:
                self.target_idx = i
                self.found_target = True
                found = True
                break
        if not found:
            self.found_target = False
            self.target_idx = -1

    def centers_cb(self, msg: Float32MultiArray):
        if self.found_target and 0 <= self.target_idx < len(msg.data):
            self.target_cx = msg.data[self.target_idx]

    def dists_cb(self, msg: Float32MultiArray):
        pass

    def get_angle_error(self):
        pixel_error = (IMG_WIDTH / 2) - self.target_cx
        angle = math.atan2(pixel_error, FOCAL_LENGTH)
        return angle
    # [수정 5] Bark 난사 콜백 함수
    def aggressive_bark_cb(self):
        msg = String()
        msg.data = "bark"
        self.speech_pub.publish(msg) # robot_dog/speech로 쏴서 덮어쓰기
        self.bark_pub.publish(msg)   # /bark 로도 전송

    def send_goal(self, x, y, yaw):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(x); goal.pose.position.y = float(y)
        goal.pose.orientation.z = math.sin(yaw/2.0); goal.pose.orientation.w = math.cos(yaw/2.0)
        self.goal_pub.publish(goal)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    # -------------------------
    # Main Loop
    # -------------------------
    def control_loop(self):
        cmd = Twist()

        # [1] 관측 위치로 이동
        if self.state == "INIT":
            self.get_logger().info(f">>> Moving to Center Spot ({OBS_X}, {OBS_Y})...")
            self.send_goal(OBS_X, OBS_Y, OBS_YAW)
            self.state = "OBSERVE"
            self.arrival_wait = True
            self.observe_start_time = 0.0

        # [2] [수정] 도착 후 "진득하게" 바라보기 (Observe)
        # 즉시 회전하지 않고, 2초 동안 정면을 보며 타겟을 찾습니다.
        elif self.state == "OBSERVE":
            if self.arrival_wait:
                return # 아직 이동 중

            if self.observe_start_time == 0.0:
                self.stop_robot()
                self.get_logger().info(">>> Arrived! Observing Candidates...")
                self.observe_start_time = time.time()
            
            # 2초 동안 대기하며 인식
            if time.time() - self.observe_start_time < 2.0:
                # 인식은 콜백(labels_cb)에서 계속 도는 중
                pass
            else:
                # 2초 후 판단
                if self.found_target:
                    self.get_logger().info(f">>> Found {self.target_color}! Aligning...")
                    self.state = "ALIGN"
                else:
                    self.get_logger().warn(">>> Not found in front. Searching (Rotating)...")
                    self.state = "SEARCH_ROT" # 정면에 없으면 그때 회전

        # [2-1] 정면에 없을 때만 회전 탐색
        elif self.state == "SEARCH_ROT":
            if self.found_target:
                self.stop_robot()
                self.state = "ALIGN"
            else:
                cmd.angular.z = 0.3
                self.cmd_pub.publish(cmd)

        # [3] 정밀 정렬 (Perfect Align)
        elif self.state == "ALIGN":
            if not self.found_target:
                self.state = "SEARCH_ROT" # 놓치면 다시 찾기
                return

            angle_err = self.get_angle_error()
            
            if abs(angle_err) < 0.035: # 2도 이내 정렬
                self.stop_robot()
                self.get_logger().info(">>> Aligned! Starting Blind Move...")
                
                # [거리 계산] 현재 위치 Y(13.5) -> 콘 Y(15.25)
                # 이동 거리 = (15.25 - 13.5) - 0.5(Stop_dist) = 1.25m
                # 안전하게 계산된 값 사용
                #move_dist = (CONE_Y - OBS_Y) - STOP_DIST
                move_dist = (CONE_Y - OBS_Y)
                if move_dist < 0: move_dist = 0.5 # 예외처리
                
                self.approach_duration = move_dist / FORWARD_SPEED
                self.approach_start_time = time.time()
                self.state = "BLIND_MOVE"
                
            else:
                # Visual Servoing
                cmd.angular.z = 1.5 * angle_err
                cmd.angular.z = max(min(cmd.angular.z, 0.5), -0.5)
                self.cmd_pub.publish(cmd)

        # [4] 맹목적 전진 (Blind Move)
        elif self.state == "BLIND_MOVE":
            elapsed = time.time() - self.approach_start_time
            
            if elapsed < self.approach_duration:
                cmd.linear.x = FORWARD_SPEED
                cmd.angular.z = 0.0 
                self.cmd_pub.publish(cmd)
            else:
                self.stop_robot()
                self.get_logger().info(">>> Reached Goal! Barking!")
                self.state = "BARK"
                self.bark_start_time = time.time()

        # [5] 짖기 및 종료
        elif self.state == "BARK":
            # 타이머가 없으면 생성 (0.001초 간격) -> Perception None 덮어쓰기용
            if self.aggressive_bark_timer is None:
                self.aggressive_bark_timer = self.create_timer(0.001, self.aggressive_bark_cb)
            
            # 4초 동안 유지 후 종료
            if time.time() - self.bark_start_time > 10.0:
                self.get_logger().info(">>> Mission Complete. Shutting Down.")
                self.state = "END"

        # [6] [수정 7] 자동 종료
        elif self.state == "END":
            # 타이머 정리
            if self.aggressive_bark_timer is not None:
                self.aggressive_bark_timer.cancel()
            
            self.stop_robot()
            time.sleep(1.0)
            raise SystemExit # 런치파일 종료 트리거

def main(args=None):
    rclpy.init(args=args)
    node = Mission3ConeDiagonal()
    
    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.logging.get_logger("mission3").info("Mission finished cleanly.")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()