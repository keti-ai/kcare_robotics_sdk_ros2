import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable

def generate_launch_description():
    # 1. 로봇 이름 인자를 선언합니다.
    param_name = 'kcare_1.yaml'
    
    param_file = os.path.join(get_package_share_directory('kcare_robot_ros2_controller'),
                              'config', param_name)
    
    xacro_file = os.path.join(get_package_share_directory('kcare_description'), 'robots',
                                     'device_elevation.urdf.xacro')

    robot_description = Command(
        [FindExecutable(name='xacro'), ' ', xacro_file])
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            namespace='elevation',
            parameters=[{'robot_description': robot_description}],
        ),
        # LM control node
        Node(
            package='kcare_robot_ros2_controller',
            executable='lm_control_node',
            name='lm_control_node',
            output='screen',
            parameters=[param_file],
        ),
    ])