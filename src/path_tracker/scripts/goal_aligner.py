#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import math

class GoalAligner(Node):
    def __init__(self):
        super().__init__('goal_aligner')
        
        # 구독
        self.create_subscription(PoseStamped, '/go1_pose', self.pose_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        # 발행 (MPPI를 이기기 위해 큐 사이즈를 1로 줄이고, 자주 쏨)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        
        self.current_pose = None
        self.goal_pose = None
        
        # 타이머 주기를 0.05초(20Hz) -> 0.02초(50Hz)로 더 빠르게 변경
        self.timer = self.create_timer(0.02, self.control_loop) 
        
        # 파라미터
        self.ARRIVE_DIST = 0.50   # 20cm 이내면 도착으로 간주
        self.ALIGN_TOLERANCE = 0.05 # 3도 이내면 정렬 완료

        self.is_aligning = False
        self.get_logger().info("Goal Aligner Node Started! Waiting for Goal...")

    def pose_callback(self, msg):
        self.current_pose = msg

    def goal_callback(self, msg):
        self.goal_pose = msg
        self.is_aligning = True
        self.get_logger().info(f"New Goal Received! x:{msg.pose.position.x:.2f}, y:{msg.pose.position.y:.2f}")

    def get_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        # 1. 데이터 수신 확인
        if self.current_pose is None or self.goal_pose is None:
            # 데이터가 안 들어오면 1초에 한 번만 로그 찍기 (도배 방지)
            # self.get_logger().info("Waiting for topics...", throttle_duration_sec=1.0)
            return

        # 2. 거리 계산
        dx = self.goal_pose.pose.position.x - self.current_pose.pose.position.x
        dy = self.goal_pose.pose.position.y - self.current_pose.pose.position.y
        dist = math.sqrt(dx**2 + dy**2)

        # [디버깅] 현재 거리와 상태를 0.5초마다 출력
        # self.get_logger().info(f"Dist: {dist:.3f}, Aligning: {self.is_aligning}", throttle_duration_sec=0.5)

        # 3. 도착 범위 진입?
        if dist < self.ARRIVE_DIST:
            
            curr_yaw = self.get_yaw(self.current_pose.pose.orientation)
            goal_yaw = self.get_yaw(self.goal_pose.pose.orientation)
            
            diff = goal_yaw - curr_yaw
            while diff > math.pi: diff -= 2 * math.pi
            while diff < -math.pi: diff += 2 * math.pi

            if abs(diff) > self.ALIGN_TOLERANCE:
                # [중요] MPPI가 멈추라고 해도, 나는 돌라고 계속 명령을 쏨!
                twist = Twist()
                ang_vel = diff * 2.0 # P 게인 키움
                twist.angular.z = max(min(ang_vel, 0.8), -0.8) # 속도 제한 0.8
                
                self.cmd_pub.publish(twist)
                
                # 회전 중임을 로그로 확인
                self.get_logger().info(f"ROTATING >> Diff: {diff:.2f} rad", throttle_duration_sec=0.2)
            else:
                if self.is_aligning:
                    # 정렬 완료 시 정지 명령
                    stop_msg = Twist()
                    self.cmd_pub.publish(stop_msg)
                    self.get_logger().info("SUCCESS: Aligned & Finished!")
                    self.get_logger().info(
                        f"  [Goal] x: {self.goal_pose.pose.position.x:.3f}, y: {self.goal_pose.pose.position.y:.3f}, yaw: {goal_yaw:.3f} rad"
                    )
                    self.get_logger().info(
                        f"  [Curr] x: {self.current_pose.pose.position.x:.3f}, y: {self.current_pose.pose.position.y:.3f}, yaw: {curr_yaw:.3f} rad"
                    )
                    self.get_logger().info(
                        f"  [Err ] dist: {dist:.3f} m, yaw_diff: {diff:.3f} rad"
                    )
                    self.is_aligning = False

def main(args=None):
    rclpy.init(args=args)
    node = GoalAligner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()