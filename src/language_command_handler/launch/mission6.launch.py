import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='language_command_handler',
            executable='mission6_nurse.py',  # CMakeLists.txt에 등록된 이름
            name='mission6_nurse_node',
            output='screen'
        )
    ])