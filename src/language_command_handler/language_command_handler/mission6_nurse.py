#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Bool, Float32MultiArray
import math
import time

# =========================================================
# [설정] 휴게실(Break Room) 진입 위치 (맵에서 확인 필요)
# 소파와 간호사가 보일만한 '방 입구' 또는 '방 중앙' 좌표
# =========================================================
APPROACH_X = -7.655    # [수정 필요] 방 입구/중앙 X
APPROACH_Y = -25.973     # [수정 필요] 방 입구/중앙 Y
APPROACH_YAW = -3.033  # [수정 필요] 방 안쪽을 바라보는 각도 (rad)
# [설정] 오르빗(Orbit) 파라미터
ORBIT_RADIUS = 1.3   # 간호사로부터 유지할 거리 (m)
ORBIT_POINTS = 8     # 다각형 꼭짓점 개수 (8각형)
TARGET_LABEL = "nurse"
# =========================================================

def yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)

def quaternion_to_yaw(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class Mission6Nurse(Node):
    def __init__(self):
        super().__init__('mission6_nurse')
        
        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10) # 회전 탐색용
        
        # Subscribers
        self.goal_reached_sub = self.create_subscription(Bool, '/goal_reached', self.goal_reached_cb, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/go1_pose', self.pose_cb, 10)
        self.labels_sub = self.create_subscription(String, '/detections/labels', self.labels_cb, 10)
        self.dists_sub = self.create_subscription(Float32MultiArray, '/detections/distances', self.dists_cb, 10)

        # Robot State
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        # Perception State
        self.found_nurse = False
        self.nurse_dist = 0.0
        self.nurse_map_x = 0.0
        self.nurse_map_y = 0.0

        # Mission State
        self.state = "INIT"  # INIT -> APPROACH -> SEARCH -> CALC -> ORBIT -> END
        self.orbit_waypoints = []
        self.current_wp_idx = 0
        self.arrival_wait = False
        
        # 타이머
        self.timer = self.create_timer(0.2, self.control_loop)
        self.search_start_time = 0.0
        
        self.get_logger().info("Mission 6 Started: Find Static Nurse & Orbit")

    # -----------------------------------------------------
    # Callbacks
    # -----------------------------------------------------
    def pose_cb(self, msg: PoseStamped):
        self.robot_x = msg.pose.position.x
        self.robot_y = msg.pose.position.y
        self.robot_yaw = quaternion_to_yaw(msg.pose.orientation)

    def goal_reached_cb(self, msg: Bool):
        if msg.data and self.arrival_wait:
            self.arrival_wait = False

    def labels_cb(self, msg: String):
        if self.state == "SEARCH":
            labels = [l.strip() for l in msg.data.split(',')]
            if TARGET_LABEL in labels:
                self.found_nurse = True

    def dists_cb(self, msg: Float32MultiArray):
        if self.state == "SEARCH" and self.found_nurse:
            dists = msg.data
            # 간호사 거리 추정 (0.5m ~ 6.0m 사이의 유효값 중 최소값)
            valid = [d for d in dists if 0.5 < d < 6.0]
            if valid:
                self.nurse_dist = min(valid)

    # -----------------------------------------------------
    # Control Loop
    # -----------------------------------------------------
    def control_loop(self):
        # [1] 방 입구로 이동
        if self.state == "INIT":
            self.get_logger().info(f"Moving to Break Room: ({APPROACH_X}, {APPROACH_Y})")
            self.send_goal(APPROACH_X, APPROACH_Y, APPROACH_YAW)
            self.state = "APPROACH"
            self.arrival_wait = True

        # [2] 도착 대기 -> 탐색 시작
        elif self.state == "APPROACH":
            if not self.arrival_wait:
                self.get_logger().info("Arrived. Starting Visual Search...")
                self.state = "SEARCH"
                self.found_nurse = False
                self.search_start_time = time.time()

        # [3] 제자리 회전하며 간호사 찾기
        elif self.state == "SEARCH":
            # A. 간호사를 찾았는가?
            if self.found_nurse and self.nurse_dist > 0.0:
                self.stop_robot()
                # 좌표 계산 (Localizing)
                self.nurse_map_x = self.robot_x + self.nurse_dist * math.cos(self.robot_yaw)
                self.nurse_map_y = self.robot_y + self.nurse_dist * math.sin(self.robot_yaw)
                
                self.get_logger().info(f"Nurse Found at ({self.nurse_map_x:.2f}, {self.nurse_map_y:.2f})")
                self.state = "CALC"
                return

            # B. 못 찾았으면 제자리 회전 (Scanning)
            # 10초 동안 천천히 돔
            if time.time() - self.search_start_time < 10.0:
                cmd = Twist()
                cmd.angular.z = 0.3  # 천천히 회전
                self.cmd_vel_pub.publish(cmd)
            else:
                self.stop_robot()
                self.get_logger().warn("Nurse not found. Assuming fixed position ahead.")
                # 타임아웃: 그냥 내 앞 2m에 있다고 가정 (미션 포기 방지)
                self.nurse_map_x = self.robot_x + 2.0 * math.cos(self.robot_yaw)
                self.nurse_map_y = self.robot_y + 2.0 * math.sin(self.robot_yaw)
                self.state = "CALC"

        # [4] 궤도(Orbit) 웨이포인트 생성
        elif self.state == "CALC":
            self.orbit_waypoints = []
            for i in range(ORBIT_POINTS):
                angle = (2 * math.pi / ORBIT_POINTS) * i
                wx = self.nurse_map_x + ORBIT_RADIUS * math.cos(angle)
                wy = self.nurse_map_y + ORBIT_RADIUS * math.sin(angle)
                self.orbit_waypoints.append((wx, wy))
            
            self.current_wp_idx = 0
            self.state = "ORBIT"
            self.get_logger().info("Orbit Path Generated. Starting Orbit...")

        # [5] 궤도 순회
        elif self.state == "ORBIT":
            if not self.arrival_wait:
                if self.current_wp_idx >= len(self.orbit_waypoints):
                    self.get_logger().info("Orbit Complete! Mission Success.")
                    self.state = "END"
                    return

                # 다음 점으로 이동
                tx, ty = self.orbit_waypoints[self.current_wp_idx]
                
                # 다음 웨이포인트를 바라보게 Yaw 설정 (Tangent)
                next_idx = (self.current_wp_idx + 1) % len(self.orbit_waypoints)
                nx, ny = self.orbit_waypoints[next_idx]
                target_yaw = math.atan2(ny - ty, nx - tx)

                self.send_goal(tx, ty, target_yaw)
                self.current_wp_idx += 1
                self.arrival_wait = True

        # [6] 종료
        elif self.state == "END":
            raise SystemExit

    # -----------------------------------------------------
    # Helper Functions
    # -----------------------------------------------------
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

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = Mission6Nurse()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()