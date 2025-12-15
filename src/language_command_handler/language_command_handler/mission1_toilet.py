#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool

# =========================================================
# [설정] 화장실 위치 좌표 (Rviz에서 확인한 값으로 수정 필수)
# =========================================================
TOILET_X = -7.1
TOILET_Y = -1.7
TOILET_YAW = 1.52
# =========================================================

TOPIC_GOAL = "/goal_pose"
TOPIC_GOAL_REACHED = "/goal_reached"
TOPIC_SPEECH = "/robot_dog/speech"

BARK_PERIOD = 0  # seconds: 계속 bark 유지 (0.2~1.0 사이 추천)

def yaw_to_quaternion(yaw):
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return qx, qy, qz, qw

class Mission1Toilet(Node):
    def __init__(self):
        super().__init__("mission1_toilet")

        self.goal_pub = self.create_publisher(PoseStamped, TOPIC_GOAL, 10)
        self.speech_pub = self.create_publisher(String, TOPIC_SPEECH, 10)

        self.goal_reached_sub = self.create_subscription(
            Bool, TOPIC_GOAL_REACHED, self.goal_reached_callback, 10
        )

        self.goal_sent = False
        self.barked = False
        self.bark_timer = None

        self.timer = self.create_timer(1.0, self.send_goal_once)
        self.get_logger().info("Mission 1: Going to Toilet...")

    def send_goal_once(self):
        if self.goal_sent:
            return

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

        self.goal_pub.publish(goal)
        self.get_logger().info(f"Published Goal: x={TOILET_X}, y={TOILET_Y}, yaw={TOILET_YAW}")

        self.goal_sent = True
        self.timer.cancel()

    def _publish_bark(self):
        m = String()
        m.data = "bark"
        self.speech_pub.publish(m)

    def goal_reached_callback(self, msg: Bool):
        # 이미 bark 모드면 이후 신호는 전부 무시
        if self.barked:
            return

        # 최초 1회만 bark 모드로 진입
        if msg.data and self.goal_sent:
            self.get_logger().info(">>> Goal Reached! Entering continuous BARK mode (ignore future signals).")
            self.barked = True

            # 즉시 1번 bark
            self._publish_bark()

            # 이후 계속 bark 유지 (주기적으로 publish)
            self.bark_timer = self.create_timer(BARK_PERIOD, self._publish_bark)

def main(args=None):
    rclpy.init(args=args)
    node = Mission1Toilet()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
