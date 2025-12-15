'''import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='language_command_handler',
            executable='mission3_cone.py',
            name='mission3_cone_node',
            output='screen',
            parameters=[{'target_color': 'blue'}] # <--- 여기만 다름
        )
    ])'''
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='language_command_handler',
            
            # [수정] 실행할 파일 이름을 새로 만든 'Pivot' 스크립트로 변경
            #executable='mission3_cone.py',  
            #executable='mission3_cone_pivot.py',  
            executable='mission3_cone_diagonal.py',  
            
            name='mission3_cone_pivot_node',
            output='screen',
            
            # [유지] 색상 파라미터는 그대로 'red', 'blue', 'green' 등으로 유지
            parameters=[{'target_color': 'blue'}] 
        )
    ])