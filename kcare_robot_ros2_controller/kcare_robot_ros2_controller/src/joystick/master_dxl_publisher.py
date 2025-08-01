import rclpy

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from kcare_robot_ros2_controller.src.master.kcare_utils import RobotParam
from kcare_robot_ros2_controller.src.joystick.master_dxl_utils.dxl_wrapper import *

from sensor_msgs.msg import JointState

import time

JOINT_NAMES = [
    "joint_1", "joint_2", "joint_3", "joint_4",
    "joint_5", "joint_6", "joint_7", "gripper_joint"
]


class KcareRobotRemoteControlNode(Node):
    def __init__(self):
        super().__init__('kcare_master')
        self.driver = None
        self.dxl_init()
        
        self.joint_state_publisher = self.create_publisher(JointState,'/master/joint_states',1)
        
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def dxl_init(self):
        self.driver = RemoteDynamixelWrapper('/dev/ttyUSB0', 57600, [1, 2, 3, 4, 5, 6, 7, 8])
        self.driver.set_target_current(30)
        self.driver.set_torque_mode(True)
        target_pose = RobotParam.arm_ready
        target_pose.append(0.0)
        self.driver.set_target_joint(target_pose)


    def timer_callback(self):
        target_robot_angle, target_robot_gripper=self.driver.read_only_joints()
        
        self.get_logger().info(f"Target Angle : {target_robot_angle}, Target Gripper : {target_robot_gripper}")

        joint_msgs=JointState()
        joint_msgs.header.stamp = self.get_clock().now().to_msg()
        joint_msgs.name = JOINT_NAMES

        joint_msgs.position = list(target_robot_angle) + [target_robot_gripper] 
        
        self.joint_state_publisher.publish(joint_msgs)

    def terminator(self):
        self.driver.set_torque_mode(False)
        self.driver.close()

def main(args=None):
    rclpy.init(args=args)
    node = KcareRobotRemoteControlNode()
    
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    # 노드내 함수 비동기 실행.초기화 함수등 실행
    #executor.create_task(node.rb_init)
    try:
        node.get_logger().info("✅ Master Server is running...")
        executor.spin()  # ✅ 멀티스레드 실행
    except KeyboardInterrupt:
        node.terminator()
    finally:
        #node.terminator()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
