import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='language_command_handler',
            executable='mission3_cone.py',
            name='mission3_cone_node',
            output='screen',
            parameters=[{'target_color': 'green'}] # <--- 여기만 다름
        )
    ])