#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
import math
import time

# =========================================================
# [CONFIGURATION]
# =========================================================
# Point A: check for stop sign
COORD_A = {'x': 5.0, 'y': 10.0, 'yaw': 0.78}

# Point B: go here if NO sign
COORD_B = {'x': 8.0, 'y': 15.0, 'yaw': -1.57}

# Point C: go here if sign IS found
COORD_C = {'x':  -8.0, 'y': 15.0, 'yaw': 1.57}

# Perception
TOPIC_LABELS = "/detections/labels"   # String CSV: "box,sign,..." or "None"
TARGET_LABEL = "sign"

# Check policy
CHECK_DURATION = 3.0   # seconds to observe at A
CHECK_MIN_HITS = 2     # at least N detections within duration => sign present
# =========================================================


def yaw_to_quaternion(yaw: float):
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return qx, qy, qz, qw


def parse_labels_csv(s: str):
    s = (s or "").strip()
    if (not s) or (s.lower() == "none"):
        return []
    return [t.strip() for t in s.split(",") if t.strip()]


class Mission5Sign(Node):
    def __init__(self):
        super().__init__("mission5_sign")

        # Publishers / Subscribers
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.goal_reached_sub = self.create_subscription(
            Bool, "/goal_reached", self.goal_reached_callback, 10
        )
        self.perception_sub = self.create_subscription(
            String, TOPIC_LABELS, self.perception_callback, 10
        )

        # State:
        # 0 idle, 1 moving to A, 2 checking at A, 3 moving to B/C
        self.state = 0

        # Check stats
        self.sign_hits = 0
        self.check_timer = None

        self.get_logger().info("Mission5 initialized.")
        self.start_timer = self.create_timer(1.0, self.start_mission)

    def start_mission(self):
        if self.state != 0:
            return
        self.get_logger().info("Moving to Point A")
        self.send_goal(COORD_A)
        self.state = 1
        self.start_timer.cancel()

    def send_goal(self, coord):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(coord["x"])
        goal.pose.position.y = float(coord["y"])

        qx, qy, qz, qw = yaw_to_quaternion(float(coord["yaw"]))
        goal.pose.orientation.x = qx
        goal.pose.orientation.y = qy
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw

        self.goal_pub.publish(goal)

    def perception_callback(self, msg: String):
        # Only count detections while checking at A
        if self.state != 2:
            return
        labels = parse_labels_csv(msg.data)
        if TARGET_LABEL in labels:
            self.sign_hits += 1
            self.get_logger().info(
                f"[CHECK] sign detected (hits={self.sign_hits})",
                throttle_duration_sec=0.5
            )

    def goal_reached_callback(self, msg: Bool):
        if not msg.data:
            return

        # Arrived at A -> start checking
        if self.state == 1:
            self.get_logger().info("Arrived at Point A. Checking for sign...")
            self.state = 2
            self.sign_hits = 0
            self.check_timer = self.create_timer(CHECK_DURATION, self.make_decision)

        # Arrived at final destination
        elif self.state == 3:
            self.get_logger().info("Arrived at final destination. Mission complete.")
            raise SystemExit

    def make_decision(self):
        if self.check_timer:
            self.check_timer.cancel()

        self.state = 3
        if self.sign_hits >= CHECK_MIN_HITS:
            self.get_logger().info(
                f"DECISION: sign found (hits={self.sign_hits}) -> Go C"
            )
            self.send_goal(COORD_C)
        else:
            self.get_logger().info(
                f"DECISION: no sign (hits={self.sign_hits}) -> Go B"
            )
            self.send_goal(COORD_B)


def main(args=None):
    rclpy.init(args=args)
    node = Mission5Sign()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
