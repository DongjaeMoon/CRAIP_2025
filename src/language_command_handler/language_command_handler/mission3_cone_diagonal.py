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
APPROACH_DRIVE_DIST = 0.2
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
                self.send_goal(goal_x, goal_y, 1.55)
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

'''#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Bool, Float32MultiArray
import math
import time

# =========================================================
# [설정] 파라미터
# =========================================================
OBS_X = 1.25
OBS_Y = 13.5
OBS_YAW = 1.57

# 콘들의 실제 맵 좌표 (고정값)
CONE_COORDS = [
    {'x': 0.1,  'y': 15.25},  # [0] Left
    {'x': 1.25, 'y': 15.25},  # [1] Center
    {'x': 2.5,  'y': 15.25}   # [2] Right
]

# 접근 설정
STOP_DIST = 0.5       # 콘 앞 0.5m 에서 정지
FORWARD_SPEED = 0.2   # 전진 속도 (m/s)

IMG_WIDTH = 640
FOV_DEG = 80.0        
FOCAL_LENGTH = (IMG_WIDTH / 2) / math.tan(math.radians(FOV_DEG / 2))
# =========================================================

class Mission3ConeDiagonal(Node):
    def __init__(self):
        super().__init__('mission3_cone_diagonal')

        self.declare_parameter('target_color', 'red')
        self.target_color = self.get_parameter('target_color').get_parameter_value().string_value.lower()
        self.get_logger().info(f">>> [Align & Blind] Target: {self.target_color.upper()}")

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
        
        # Blind Approach Variables
        self.approach_start_time = 0.0
        self.approach_duration = 0.0
        
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
        # 이번 로직에선 perception 거리는 참고만 하거나 안 씀
        pass

    def get_angle_error(self):
        pixel_error = (IMG_WIDTH / 2) - self.target_cx
        angle = math.atan2(pixel_error, FOCAL_LENGTH)
        return angle

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

        # [1] 이동
        if self.state == "INIT":
            self.get_logger().info(f"Moving to Obs Spot ({OBS_X}, {OBS_Y})...")
            self.send_goal(OBS_X, OBS_Y, OBS_YAW)
            self.state = "SCAN"
            self.arrival_wait = True

        # [2] 스캔
        elif self.state == "SCAN":
            if not self.arrival_wait:
                if self.found_target and self.target_cx > 0:
                    self.stop_robot()
                    self.get_logger().info(f">>> Found {self.target_color}! Aligning...")
                    self.state = "ALIGN"
                else:
                    cmd.angular.z = 0.3
                    self.cmd_pub.publish(cmd)

        # [3] 정밀 정렬 (Perfect Align)
        elif self.state == "ALIGN":
            if not self.found_target:
                self.state = "SCAN"
                return

            angle_err = self.get_angle_error()
            
            if abs(angle_err) < 0.035: # 2도 이내 정렬
                self.stop_robot()
                self.get_logger().info(">>> Aligned! Calculating Blind Distance...")
                
                # [수정] 정렬 직후, Perception 거리 대신 '하드코딩된 거리' 계산
                # 현재 로봇은 OBS_Y(14.5)에 있고, 콘은 Y=15.25에 있음.
                # X축 정렬은 맞췄다고 가정.
                # 남은 거리 = (콘 Y) - (현재 Y) - (멈출 거리)
                # 근데 로봇 위치가 정확하지 않을 수 있으니, 그냥 안전하게 0.5m 전진하거나
                # 혹은 Perception 거리의 '마지막 값'을 스냅샷 떠서 쓸 수도 있음.
                
                # 여기서는 사용자 요청대로 '하드코딩 식' 접근 (시간 기반)
                # OBS_Y(14.5) -> CONE_Y(15.25) : 차이는 0.75m
                # 거기서 STOP_DIST(0.5m)를 빼면 0.25m 정도만 더 가면 됨.
                # 하지만 로봇 팔 길이 등 고려해서 0.4m 정도 전진하도록 설정.
                
                move_dist = 1.5  # [하드코딩] 40cm 전진
                
                self.approach_duration = move_dist / FORWARD_SPEED
                self.approach_start_time = time.time()
                self.state = "BLIND_MOVE"
                
            else:
                cmd.angular.z = 1.5 * angle_err
                cmd.angular.z = max(min(cmd.angular.z, 0.5), -0.5)
                self.cmd_pub.publish(cmd)

        # [4] [수정] 맹목적 전진 (Blind Move)
        elif self.state == "BLIND_MOVE":
            elapsed = time.time() - self.approach_start_time
            
            if elapsed < self.approach_duration:
                # Perception 무시하고 그냥 앞으로 감
                cmd.linear.x = FORWARD_SPEED
                # 직진성 유지 (회전 없음)
                cmd.angular.z = 0.0 
                self.cmd_pub.publish(cmd)
            else:
                self.stop_robot()
                self.get_logger().info(">>> Blind Move Done. Barking!")
                self.state = "BARK"

        # [5] 짖기
        elif self.state == "BARK":
            msg = String(); msg.data = "bark"
            self.bark_pub.publish(msg)
            self.speech_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Mission3ConeDiagonal()
    try: rclpy.spin(node)
    except: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()'''