import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='language_command_handler',
            executable='mission1_toilet.py',  # CMakeLists.txt에 등록된 이름
            name='mission1_toilet_node',
            output='screen'
        )
    ])