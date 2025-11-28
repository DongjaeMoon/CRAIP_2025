"""
Path Tracker Launch File

This launch file starts the path tracker node with MPPI controller configuration.
The node subscribes to path commands and publishes velocity commands to control
the GO1 robot along the desired path.

Usage:
    ros2 launch path_tracker path_tracker_launch.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Generate launch description for path tracker node.
    
    Returns:
        LaunchDescription: Launch description containing the path tracker node
    """
    # Get path tracker package directory
    path_tracker_pkg_dir = get_package_share_directory('path_tracker')
    
    # Path to MPPI configuration file
    mppi_config_path = os.path.join(path_tracker_pkg_dir, 'config', 'mppi.yaml')
    
    # Create path tracker node
    path_tracker_node = Node(
        package='path_tracker',
        executable='path_tracker_node',
        name='path_tracker',
        output='screen',
        parameters=[mppi_config_path],
        emulate_tty=True
    )

    # 2. [추가됨] Goal Aligner (Python Script)
    # CMakeLists.txt에서 install(PROGRAMS ...)로 설치했으므로
    # executable 이름에 .py까지 적어주면 실행됩니다.
    '''
    goal_aligner_node = Node(
        package='path_tracker',
        executable='goal_aligner.py',  # .py 확장자 포함 중요!
        name='goal_aligner',
        output='screen',
        emulate_tty=True
    )'''

    return LaunchDescription([
        path_tracker_node
        #goal_aligner_node
    ])