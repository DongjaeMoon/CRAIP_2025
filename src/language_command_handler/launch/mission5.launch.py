import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Mission Node
    mission_node = Node(
        package='language_command_handler', # <--- CHANGE THIS to your actual package name
        executable='mission5_sign',
        name='mission5_sign',
        output='screen'
    )

    return LaunchDescription([
        mission_node
    ])