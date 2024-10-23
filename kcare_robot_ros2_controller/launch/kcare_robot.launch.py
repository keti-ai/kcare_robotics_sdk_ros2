#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    xarm_driver_launch_file = os.path.join(
        get_package_share_directory('xarm_api'),
        'launch',
        'xarm7_driver.launch.py'
    )
    
    # femto_driver_launch_file = os.path.join(
    #     get_package_share_directory('azure_kinect_ros_driver'),
    #     'launch',
    #     'driver.launch.py',
    # )
    
    # hand_driver_launch_file = os.path.join(
    #     get_package_share_directory('orbbec_camera'),
    #     'launch',
    #     'dabai_dcw2.launch.py',
    # )

    return LaunchDescription([
        # xarm7_driver.launch.py 포함
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(xarm_driver_launch_file),
        ),

    
        # driver.launch.py 포함
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(femto_driver_launch_file),
        # ),
       
        # gemini_ew launch 포함
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(hand_driver_launch_file),
        # ),

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

        # Gripper control node
        Node(
            package='kcare_robot_ros2_controller',
            executable='gripper_control_node',
            name='gripper_control_node',
            output='screen',
            parameters=[{
                # 필요한 경우 파라미터를 추가하세요
            }]
        ),
    ])
