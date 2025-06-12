import os
import json
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration

from kcare_robot_ros2_controller.src.pyutils.config_loader import load_robot_config, get_param 

def generate_launch_description():
    config_package_name = 'kcare_robot_ros2_controller' 
    robot_config = load_robot_config(
        package_name=config_package_name,
        config_file_env_var='ROBOT_NAME',
        default_robot_name='default'
    )
    
    # 기본값 설정
    femto_serial = get_param(robot_config,['camera','femto_serial'],'')
    femto_port = get_param(robot_config,['camera','femto_port'],'')
    gemini_serial = get_param(robot_config,['camera','gemini_serial'],'')
    gemini_port = get_param(robot_config,['camera','gemini_port'],'')

    print(f"[LAUNCH] Loaded Femto serial: {femto_serial}") # 런치 시스템 로그 대신 직접 출력
    print(f"[LAUNCH] Loaded Femto port: {femto_port}") # 런치 시스템 로그 대신 직접 출력
    print(f"[LAUNCH] Loaded Gemini serial: {gemini_serial}") # 런치 시스템 로그 대신 직접 출력
    print(f"[LAUNCH] Loaded Gemini port: {gemini_port}") # 런치 시스템 로그 대신 직접 출력

    # 4. Orbbec 카메라 런치 파일의 경로 설정
    orbbec_package_dir = get_package_share_directory('orbbec_camera')
    orbbec_launch_file_dir = os.path.join(orbbec_package_dir, 'launch')

    # 5. Femto Bolt 카메라 런치 설정
    launch1_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(orbbec_launch_file_dir, 'femto_bolt.launch.py')
        ),
        launch_arguments={
            'camera_name': 'femto',
            'usb_port': femto_port,    # JSON에서 로드된 값
            'serial_number': femto_serial, # JSON에서 로드된 값
            'device_num': '2', # 이 값은 config 파일에서 관리하지 않는다면 고정 또는 다른 방법으로 관리
            'depth_registration': 'true',
            'color_fps': '15',
            'depth_fps': '15',
            'enable_point_cloud': 'true',
            'publish_tf': 'false',
            'cloud_frame_id': 'femto_depth_optical_frame',
            'enable_colored_point_cloud': 'false',
        }.items()
    )

    # 6. Gemini2 카메라 런치 설정
    launch2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(orbbec_launch_file_dir, 'gemini2.launch.py')
        ),
        launch_arguments={
            'camera_name': 'hand',
            'usb_port': gemini_port,    # JSON에서 로드된 값
            'serial_number': gemini_serial, # JSON에서 로드된 값
            'device_num': '2', # 이 값은 config 파일에서 관리하지 않는다면 고정 또는 다른 방법으로 관리
            'depth_registration': 'true',
            'color_fps': '5',
            'depth_fps': '5',
            'enable_point_cloud': 'true',
            'publish_tf': 'false',
            'cloud_frame_id': 'hand_depth_optical_frame',
            'enable_colored_point_cloud': 'false',
        }.items()
    )

    # Launch description
    ld = LaunchDescription([
        GroupAction([launch1_include]),
        GroupAction([launch2_include]),
    ])

    return ld