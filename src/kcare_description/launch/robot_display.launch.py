#  Copyright (c) 2021 Franka Emika GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


# def generate_launch_description():
#     robot_xacro_file = os.path.join(get_package_share_directory('kcare_description'), 'robots',
#                                      'kcare_platform.urdf.xacro')
#     robot_description = Command(
#         [FindExecutable(name='xacro'), ' ', robot_xacro_file])


#     rviz_file = os.path.join(get_package_share_directory('kcare_description'), 'rviz',
#                              'robot_config.rviz')

#     return LaunchDescription([
#         Node(
#             package='robot_state_publisher',
#             executable='robot_state_publisher',
#             name='robot_state_publisher',
#             output='screen',
#             namespace='',
#             parameters=[{'robot_description': robot_description}],
#         ),

#         Node(
#             package='joint_state_publisher_gui',
#             executable='joint_state_publisher_gui',
#             name='joint_state_publisher_gui',
#             namespace='',
#         ),

#         Node(package='rviz2',
#              executable='rviz2',
#              name='rviz2',
#              arguments=['--display-config', rviz_file])
#     ])

def generate_launch_description():
    slam_xacro_file = os.path.join(get_package_share_directory('kcare_description'), 'robots',
                                     'device_mobile.urdf.xacro')
    slam_description = Command(
        [FindExecutable(name='xacro'), ' ', slam_xacro_file])

    elevation_xacro_file = os.path.join(get_package_share_directory('kcare_description'), 'robots',
                                     'device_elevation.urdf.xacro')
    elevation_description = Command(
        [FindExecutable(name='xacro'), ' ', elevation_xacro_file])

    xarm_xacro_file = os.path.join(get_package_share_directory('kcare_description'), 'robots',
                                     'device_xarm7.urdf.xacro')
    xarm_description = Command(
        [FindExecutable(name='xacro'), ' ', xarm_xacro_file])

    head_xacro_file = os.path.join(get_package_share_directory('kcare_description'), 'robots',
                                     'device_head.urdf.xacro')
    head_description = Command(
        [FindExecutable(name='xacro'), ' ', head_xacro_file])


    rviz_file = os.path.join(get_package_share_directory('kcare_description'), 'rviz',
                             'robot_config.rviz')




    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            namespace='slam',
            parameters=[{'robot_description': slam_description}],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            namespace='elevation',
            parameters=[{'robot_description': elevation_description}],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            namespace='xarm',
            parameters=[{'robot_description': xarm_description}],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            namespace='head',
            parameters=[{'robot_description': head_description}],
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            namespace='slam',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            namespace='elevation',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            namespace='xarm',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            namespace='head',
        ),

        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['--display-config', rviz_file])
    ])