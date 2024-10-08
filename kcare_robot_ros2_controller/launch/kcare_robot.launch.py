#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # # Joystick control node
        # Node(
        #     package='kcare_robot_ros2_controller',
        #     executable='kcare_robot_joy_control_node',
        #     name='kcare_robot_joy_control_node',
        #     output='screen',
        #     parameters=[{
        #         # 필요한 경우 파라미터를 추가하세요
        #     }]
        # ),

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

        # # Gripper control node
        # Node(
        #     package='kcare_robot_ros2_controller',
        #     executable='gripper_control_node',
        #     name='gripper_control_node',
        #     output='screen',
        #     parameters=[{
        #         # 필요한 경우 파라미터를 추가하세요
        #     }]
        # ),
    ])
