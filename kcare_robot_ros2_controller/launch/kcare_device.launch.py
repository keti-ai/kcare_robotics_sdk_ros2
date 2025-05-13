#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # LM control node
        Node(
            package='kcare_robot_ros2_controller',
            executable='lm_control_node',
            name='lm_control_node',
            output='screen',
            parameters=[{
                # 필요한 경우 파라미터를 추가하세요
            }]
        ),

        # Head control node
        Node(
            package='kcare_robot_ros2_controller',
            executable='head_control_node',
            name='head_control_node',
            output='screen',
            parameters=[{
                # 필요한 경우 파라미터를 추가하세요
            }]
        ),
        
        # Gripper Control Node
        Node(
            package='kcare_robot_ros2_controller',
            executable='gripper_control_node',
            name='gripper_control_node',
            output='screen',
            parameters=[{
                # 필요한 경우 파라미터를 추가하세요
            }]
        ),

        # Device Server Node
        Node(
            package='kcare_robot_ros2_controller',
            executable='device_node',
            name='device_node',
            output='screen',
            parameters=[{
                # 필요한 경우 파라미터를 추가하세요
            }]
        ),
    ])