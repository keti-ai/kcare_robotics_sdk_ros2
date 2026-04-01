import os
from glob import glob
from setuptools import setup


package_name = 'kcare_robot_ros2_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, f'launch'), glob(f'launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob(os.path.join(package_name, 'config', '*.json'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join(package_name, 'config', '*.yaml'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join(package_name, 'config', '*.stcm'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='keti',
    maintainer_email='moonjongsul@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kcare_robot_joy_control_node = kcare_robot_ros2_controller.src.joystick.kcare_robot_joy_control_node_slamware:main',
            'lm_control_node = kcare_robot_ros2_controller.src.elevation.lm_control_node:main',
            #'lm_failsafe_node = kcare_robot_ros2_controller.src.elevation.lm_failsafe:main',
            'head_control_node = kcare_robot_ros2_controller.src.head.head_control_node:main',
            'gripper_control_node = kcare_robot_ros2_controller.src.gripper.xarm_gripper_node:main',
            'virtual_gripper_node = kcare_robot_ros2_controller.src.gripper.virtual_gripper_node:main',
            'master_dxl_node = kcare_robot_ros2_controller.src.joystick.master_dxl_publisher:main',
            'remote_node = kcare_robot_ros2_controller.src.joystick.kcare_robot_master_control_node:main',
            'save_map = kcare_robot_ros2_controller.src.mobile.slam_save_map:main',
            'load_map = kcare_robot_ros2_controller.src.mobile.slam_load_map:main',
            'mobile_control_node = kcare_robot_ros2_controller.src.mobile.mobile_control_node:main',
        ],
    },
)