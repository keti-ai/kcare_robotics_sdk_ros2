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
            'gripper_control_node = kcare_robot_ros2_controller.src.gripper.gripper_node:main',
            #'master_node = kcare_robot_ros2_controller.src.master.kcare_master:main',
            'remote_node = kcare_robot_ros2_controller.src.joystick.kcare_robot_remote_control_node:main'
        ],
    },
)
