from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Include launch files
    package_dir = get_package_share_directory('xarm_api')
    launch_file_dir = os.path.join(package_dir, 'launch')
    launch1_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'xarm7_driver.launch.py')
        ),
        launch_arguments={
            'robot_ip': '192.168.2.224',
        }.items()
    )

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

    # Include launch files
    package_dir = get_package_share_directory('kcare_robot_ros2_controller')
    launch_file_dir = os.path.join(package_dir, 'launch')
    launch3_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'orbbec_camera.launch.py')
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
