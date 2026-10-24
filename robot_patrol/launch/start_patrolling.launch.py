import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='robot_patrol', executable='patrol_node', name='patrol_node', output='screen'),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen')
    ])
