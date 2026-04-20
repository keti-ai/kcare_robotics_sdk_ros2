import os
import json
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
from kcare_robot_ros2_controller.src.pyutils.config_loader import load_robot_config, get_param

def generate_launch_description():
    # Include launch files
    package_dir = get_package_share_directory('kcare_robot_ros2_controller')
    launch_file_dir = os.path.join(package_dir, 'launch')
    launch2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'kcare_device.launch.py')
        ),
        launch_arguments={
        }.items()
    )

    # Include XML launch file
    package_dir_slamware = get_package_share_directory('slamware_ros_sdk')
    launch_file_dir_slamware = os.path.join(package_dir_slamware, 'launch')
    launch4_include = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(launch_file_dir_slamware, 'slamware_ros_sdk_server_node.xml')
        )
    )

    # Define the new node to be launched
    mobile_node = Node(
        package='kcare_robot_ros2_controller',  # 여기에 실제 패키지 이름을 넣어주세요
        executable='mobile_control_node',
        name='mobile_control_node'
    )


    # Launch description
    ld = LaunchDescription([
        GroupAction([launch2_include]),
        GroupAction([launch4_include]),
        mobile_node,
    ])

    return ld
