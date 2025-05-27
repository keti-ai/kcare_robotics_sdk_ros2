#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # Joystick control node
        Node(
            package='kcare_robot_ros2_controller',
            executable='kcare_robot_joy_control_node',
            name='kcare_robot_joy_control_node',
            output='screen',
            parameters=[{
                # 필요한 경우 파라미터를 추가하세요
            }]
        ),
        # Remote Controller Node
        Node(
            package='kcare_robot_ros2_controller',
            executable='remote_node',
            name='remote_node',
            output='screen',
            parameters=[{
                # 필요한 경우 파라미터를 추가하세요
            }]
        ),
        # Joy Node for Joystick
        Node(
            package='joy',
            executable='joy_node',
            output='screen',
        ),        
    ])
