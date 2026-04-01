import os
import json
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction

from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
from kcare_robot_ros2_controller.src.pyutils.config_loader import load_robot_config, get_param



def generate_launch_description():
    xarm_ip_value = '192.168.5.212'
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

    # 2. 실행 설명(LaunchDescription) 반환
    return LaunchDescription([
        lm_control_node,
        gripper_control_node,
        head_control_node,
        xarm_launch
    ])


