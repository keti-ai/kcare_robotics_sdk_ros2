#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import Command, FindExecutable, LaunchConfiguration

def generate_launch_description():
    manipulator_xacro_file = os.path.join(get_package_share_directory('kcare_description'), 'robots',
                                     'kcare_platform.urdf.xacro')

    robot_description = Command(
        [FindExecutable(name='xacro'), ' ', manipulator_xacro_file])
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            namespace='',
            parameters=[{'robot_description': robot_description}],
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
    ])
