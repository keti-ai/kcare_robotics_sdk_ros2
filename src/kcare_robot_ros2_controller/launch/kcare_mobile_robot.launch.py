import os
import json
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction

from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
from kcare_robot_ros2_controller.src.pyutils.config_loader import load_robot_config, get_param

def generate_launch_description():
    xarm_ip_value = '192.168.5.233'
    # Include launch files
    xarm_package_dir = get_package_share_directory('xarm_api')
    xarm_launch_file_dir = os.path.join(xarm_package_dir, 'launch')
    xarm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(xarm_launch_file_dir, 'xarm7_driver.launch.py')
        ),
        launch_arguments={
            'robot_ip': xarm_ip_value, # JSON에서 로드된 IP를 사용
        }.items()
    )
    
    # Include XML launch file
    package_dir_slamware = get_package_share_directory('slamware_ros_sdk')
    launch_file_dir_slamware = os.path.join(package_dir_slamware, 'launch')
    slam_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(launch_file_dir_slamware, 'slamware_ros_sdk_server_node.xml')
        )
    )
    
    
    lm_control_node = Node(
        package='kcare_robot_ros2_controller',  # 패키지 명
        executable='lm_control_node',          # 실행 파일 명 (CMakeLists/setup.py에 정의된 이름)
        name='lm_control_node',                # 노드 이름 (옵션)
        output='screen',                       # 로그 출력 방식
    )

    gripper_control_node = Node(
        package='kcare_robot_ros2_controller',  # 패키지 명
        executable='gripper_control_node',          # 실행 파일 명 (CMakeLists/setup.py에 정의된 이름)
        name='gripper_control_node',                # 노드 이름 (옵션)
        output='screen',   
    )

    head_control_node = Node(
        package='kcare_robot_ros2_controller',  # 패키지 명
        executable='head_control_node',          # 실행 파일 명 (CMakeLists/setup.py에 정의된 이름)
        name='head_control_node',                # 노드 이름 (옵션)
        output='screen', 
    )
    
    mobile_node = Node(
        package='kcare_robot_ros2_controller',  # 여기에 실제 패키지 이름을 넣어주세요
        executable='mobile_control_node',
        name='mobile_control_node'
    )
    
    # Include launch files
    femto_package_dir = get_package_share_directory('orbbec_camera')
    femto_launch_file_dir = os.path.join(femto_package_dir, 'launch')
    
    femto_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(femto_launch_file_dir, 'femto_bolt.launch.py')
        ),
        launch_arguments={
            'camera_name': 'femto',
            'depth_registration': 'true',
            'color_qos': 'SENSOR_DATA',
            'depth_qos': 'SENSOR_DATA',
            'publish_tf': 'false',
        }.items(),
    )

    # 2. 실행 설명(LaunchDescription) 반환
    return LaunchDescription([
        lm_control_node,
        gripper_control_node,
        head_control_node,
        xarm_launch,
        slam_launch,
        mobile_node,
        TimerAction(period=2.0, actions=[GroupAction([femto_launch_include])]),
    ])