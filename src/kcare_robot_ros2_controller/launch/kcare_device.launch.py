import os
import json
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction

from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
from kcare_robot_ros2_controller.src.pyutils.config_loader import load_robot_config, get_param

def generate_launch_description():
    # Include launch files
    package_dir = get_package_share_directory('kcare_robot_ros2_controller')
    launch_file_dir = os.path.join(package_dir, 'launch')
    launch1_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'manipulator.launch.py')
        ),
        launch_arguments={
        }.items()
    )
    # Include launch files
    package_dir = get_package_share_directory('kcare_robot_ros2_controller')
    launch_file_dir = os.path.join(package_dir, 'launch')
    launch2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'elevation.launch.py')
        ),
        launch_arguments={
        }.items()
    )
    # Include launch files
    package_dir = get_package_share_directory('kcare_robot_ros2_controller')
    launch_file_dir = os.path.join(package_dir, 'launch')
    launch3_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'head.launch.py')
        ),
        launch_arguments={
        }.items()
    )

    # Launch description
    ld = LaunchDescription([
        GroupAction([launch1_include]),
        GroupAction([launch2_include]),
        GroupAction([launch3_include]),
    ])

    return ld
