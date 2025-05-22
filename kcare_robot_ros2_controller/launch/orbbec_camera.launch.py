from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Include launch files
    package_dir = get_package_share_directory('orbbec_camera')
    launch_file_dir = os.path.join(package_dir, 'launch')
    launch1_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'femto_bolt.launch.py')
        ),
        launch_arguments={
            'camera_name': 'femto',
            'usb_port': '2-1',    # Change by camera port
            'serial_number': 'CL838420024', # Change by camera serial numper
            'device_num': '2',
            'depth_registration': 'true',
            'color_fps': '15',
            'depth_fps': '15',
            'enable_point_cloud': 'true',
            'cloud_frame_id': 'femto_depth_optical_frame',
            'enable_colored_point_cloud': 'true',
        }.items()
    )

    launch2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'gemini2.launch.py')
        ),
        launch_arguments={
            'camera_name': 'hand',
            'usb_port': '1-4.2 ',    # Change by camera port
            'serial_number': 'AY37943026N',# Change by camera serial numper
            'device_num': '2',
            'depth_registration': 'true',
            'color_fps': '15',
            'depth_fps': '15',
            'enable_point_cloud': 'true',
            'cloud_frame_id': 'hand_depth_optical_frame',
            'enable_colored_point_cloud': 'true',
        }.items()
    )

    # If you need more cameras, just add more launch_include here, and change the usb_port and device_num

    # Launch description
    ld = LaunchDescription([
        GroupAction([launch1_include]),
        GroupAction([launch2_include]),
    ])

    return ld
