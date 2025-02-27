import rclpy
import rclpy.executors
from rclpy.node import Node

from sensor_msgs.msg import Image,CameraInfo
from xarm_msgs.msg import RobotMsg
from xarm_msgs.srv import MoveJoint, MoveCartesian, SetInt16, SetInt16ById, Call
from kcare_robot_ros2_controller_msgs.msg import HeadCommand, HeadState, LMCommand, LMState
from kcare_robot_ros2_controller_msgs.srv import ElevationCommand, GripperCommand

from rclpy.executors import MultiThreadedExecutor

import threading, time

SERVICE_CLIENTS ={
    'set_servo_angle' : ('/xarm/set_servo_angle',MoveJoint),
    'set_servo_move': ('/xarm/set_position', MoveCartesian),
    'motion_enable': ('/xarm/motion_enable', SetInt16),
    'set_mode': ('/xarm/set_mode', SetInt16),
    'set_state': ('/xarm/set_state', SetInt16),
    'clean_error': ('/xarm/clean_error', Call),
    'elevation_command' : ('/elevation/set_position',ElevationCommand),
    'gripper_command' : ('/gripper/command',GripperCommand),
}

TOPIC_SUBS ={
    'robot_pose' : ('/xarm/robot_states',RobotMsg),
    'lm_state' : ('/elevation/state',LMState),
    'head_state' : ('/head/state',HeadState),
}


class RobotUtils(Node):
    def __init__(self):
        super().__init__('kcare_utils')

        # 서비스 클라이언트 설정
        self.service_clients = {}
        for service_name, (service_topic, service_type) in SERVICE_CLIENTS.items():
            self.service_clients[service_name] = self.create_client(service_type, service_topic)
            self.get_logger().info(f"Service Client created: {service_name} -> {service_topic}")

        # 토픽 구독 설정


    def run_spin(self):
        rclpy.spin(self,executor=self.executor)


    def call_service(service_name, srv_type, sync=True, *args, **kwargs):
        pass



    def xarm_init(self):
        future = self.service_clients['set_state'].call_async(SetInt16.Request(data=0))
        while not future.done():
            pass

        try:
            return future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            return None

