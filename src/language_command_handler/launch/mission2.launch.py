#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="language_command_handler",
            executable="mission2_food.py",
            name="mission2_food",
            output="screen",
        )
    ])

