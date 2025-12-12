#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
import math
import sys

# =========================================================
# [CONFIGURATION] Coordinate Setup
# Modify these values to change the locations.
# =========================================================

# Location A: First check-point (Look for Stop Sign here)
COORD_A = {
    'x': -5.0, 
    'y': 10.0, 
    'yaw': 2.25  # Radians
}

# Location B: Destination if NO Stop Sign is found
COORD_B = {
    'x': -8.0, 
    'y': 15.0, 
    'yaw': -1.57
}

# Location C: Destination if Stop Sign IS found
COORD_C = {
    'x': 8.0, 
    'y': 15.0, 
    'yaw': -1.57
}

# Detection Settings
TARGET_LABEL = "sign"   # The label produced by perception_node
CHECK_DURATION = 3.0    # Time (seconds) to wait at A to observe the sign
# =========================================================

def yaw_to_quaternion(yaw):
    """Convert Yaw angle to ROS Quaternion"""
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return qx, qy, qz, qw

class Mission5Sign(Node):
    def __init__(self):
        super().__init__('mission5_sign')

        # --- Publishers & Subscribers ---
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Subscribe to Path Tracker status
        self.goal_reached_sub = self.create_subscription(
            Bool, '/goal_reached', self.goal_reached_callback, 10
        )
        
        # Subscribe to Perception Node results
        self.perception_sub = self.create_subscription(
            String, '/detections/labels', self.perception_callback, 10
        )

        # --- State Management ---
        # 0: Idle, 1: Moving to A, 2: Checking at A, 3: Moving to Final (B or C)
        self.state = 0 
        self.sign_detected = False
        
        self.get_logger().info("Mission 5 Initiated: Check Sign and Branch Path.")

        # Give system 1 second to stabilize, then start mission
        self.timer_start = self.create_timer(1.0, self.start_mission)

    def start_mission(self):
        if self.state == 0:
            self.get_logger().info(f"Step 1: Moving to Point A ({COORD_A['x']}, {COORD_A['y']})")
            self.send_goal(COORD_A)
            self.state = 1
            self.timer_start.cancel() # Stop the start timer

    def send_goal(self, coord_dict):
        """Helper to create and publish a goal pose"""
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        
        goal.pose.position.x = float(coord_dict['x'])
        goal.pose.position.y = float(coord_dict['y'])
        
        qx, qy, qz, qw = yaw_to_quaternion(coord_dict['yaw'])
        goal.pose.orientation.x = qx
        goal.pose.orientation.y = qy
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw
        
        self.goal_pub.publish(goal)

    def perception_callback(self, msg):
        """
        Continuously updates if the sign is seen.
        Only matters when we are in the 'Checking' state, but we track it anyway.
        """
        # Based on your perception_node, msg.data contains the label
        if msg.data == TARGET_LABEL:
            self.sign_detected = True
            # Optional: Log only if we are currently checking
            if self.state == 2:
                self.get_logger().info("Stop Sign Detected!", throttle_duration_sec=1.0)

    def goal_reached_callback(self, msg):
        """Called when the robot reaches a destination"""
        if not msg.data:
            return

        # Case 1: Just arrived at Point A
        if self.state == 1:
            self.get_logger().info("Arrived at Point A. Checking for sign...")
            self.state = 2 # Change state to Checking
            
            # Clear detection flag to ensure we look at fresh data
            self.sign_detected = False 
            
            # Start a timer to wait at Point A for CHECK_DURATION
            self.check_timer = self.create_timer(CHECK_DURATION, self.make_decision)

        # Case 2: Arrived at Point B or C (Mission Finished)
        elif self.state == 3:
            self.get_logger().info("Arrived at Final Destination. Mission Complete.")
            raise SystemExit

    def make_decision(self):
        """Called after waiting at Point A"""
        self.check_timer.cancel() # Stop the checking timer
        
        self.state = 3 # Next state is moving to final
        
        if self.sign_detected:
            self.get_logger().info(f"DECISION: Stop Sign Found. Moving to Point C ({COORD_C['x']}, {COORD_C['y']})")
            self.send_goal(COORD_C)
        else:
            self.get_logger().info(f"DECISION: No Sign. Moving to Point B ({COORD_B['x']}, {COORD_B['y']})")
            self.send_goal(COORD_B)

def main(args=None):
    rclpy.init(args=args)
    node = Mission5Sign()
    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.logging.get_logger("mission5_sign").info("Mission Node Exiting...")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()