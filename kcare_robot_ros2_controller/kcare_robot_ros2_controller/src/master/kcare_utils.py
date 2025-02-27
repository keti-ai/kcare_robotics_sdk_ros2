import rclpy
import rclpy.executors
from rclpy.node import Node

from sensor_msgs.msg import Image,CameraInfo
from xarm_msgs.msg import RobotMsg
from xarm_msgs.srv import MoveJoint, MoveCartesian, SetInt16, SetInt16ById, Call
from kcare_robot_ros2_controller_msgs.msg import HeadState, LMState
from kcare_robot_ros2_controller_msgs.srv import ElevationCommand, GripperCommand, HeadPoseCommand

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import time, math, copy

class RobotState:
    angle: list = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
    pose: list = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    lm_pose: float = 0.0
    head_state: float = [0.0, 0.0]

class RobotParam:
    spin_time: float = 0.05
    elev_home: float = 0.195
    arm_home: list = [math.radians(90.0),0.0,0.0,0.0,0.0,0.0,0.0]
    arm_ready: list = [math.radians(90.0),math.radians(15.0),0.0,math.radians(15.0),0.0,math.radians(-90.0),0.0]
    tool_radius: float = 50.0
    tool_length: float = 200.0


class RobotUtils(Node):
    def __init__(self):
        super().__init__('kcare_utils')

        self.topic_sub_group = MutuallyExclusiveCallbackGroup()
        self.act_callback_group = MutuallyExclusiveCallbackGroup()

        self.state=RobotState()

        SERVICE_CLIENTS ={
            'set_servo_angle' : ('/xarm/set_servo_angle',MoveJoint),
            'set_servo_move': ('/xarm/set_position', MoveCartesian),
            'motion_enable': ('/xarm/motion_enable', SetInt16),
            'set_mode': ('/xarm/set_mode', SetInt16),
            'set_state': ('/xarm/set_state', SetInt16),
            'clean_error': ('/xarm/clean_error', Call),
            'elevation_command' : ('/elevation/set_position',ElevationCommand),
            'gripper_command' : ('/gripper/command',GripperCommand),
            'head_command' : ('/head/pose_command',HeadPoseCommand),
        }

        TOPIC_SUBS = {
            'robot_pose': ('/xarm/robot_states', RobotMsg,self.robot_pose_callback),
            'lm_state': ('/elevation/state', LMState,self.elevation_state_callback),
            'head_state': ('/head/state', HeadState,self.head_state_callback),
        }

        # 서비스 클라이언트 설정
        self.service_clients = {}
        for service_tag, (service_name, service_type) in SERVICE_CLIENTS.items():
            self.service_clients[service_tag] = self.create_client(service_type, service_name)
            self.get_logger().info(f"Service Client created: {service_tag} -> {service_name}")

        # 토픽 구독 설정
        self.topic_subs = {}
        for topic_tag, (topic_name, topic_type,topic_callback_fun) in TOPIC_SUBS.items():
            self.topic_subs[topic_tag] = self.create_subscription(topic_type,topic_name,topic_callback_fun,10,callback_group=self.topic_sub_group)
            self.get_logger().info(f"Subscriber created: {topic_tag} -> {topic_name} with {topic_callback_fun}")


    def robot_pose_callback(self,msg):
        # 로봇팔 상태, 조인트 및 좌표 토픽 콜백  
        # self.get_logger().info(f'Robot State: "%s"' % str(msg.state))
        # self.get_logger().info(f'Robot Mode: "%s"' % str(msg.mode))
        # self.get_logger().info(f'Robot Error: "%s"' % str(msg.err))
        # self.get_logger().info(f'Robot Angles: "%s"' % str(msg.angle))
        # self.get_logger().info(f'Robot Pose: "%s"' % str(msg.pose))
        
        #TODO 조인트 및 포즈값 인덱스
        self.state.angle=msg.angle
        self.state.pose=msg.pose
        
        self.get_logger().info(f"rb_pose_callback node.angle: {self.state.angle}")
        # self.get_logger().info(f"rb_pose_callback node.pose: {self.state.pose}")


    def elevation_state_callback(self,msg):
        self.state.lm_current_pose=msg.current_position

    def head_state_callback(self,msg):
        self.state.head_state=[msg.current_rz,msg.current_ry]

    def call_set_mode(self, mode):
        # 모드 정의 관련 서비스 호출 예시
        '''
        Mode 0 : xArm controller (Position) mode.
        Mode 1 : External trajectory planner (position) mode.
        Mode 2 : Free-Drive (zero gravity) mode.
        Mode 3 : Reserved.
        Mode 4 : Joint velocity control mode.
        Mode 5 : Cartesian velocity control mode.
        Mode 6 : Joint space online planning mode. (Firmware >= v1.10.0)
        Mode 7 : Cartesian space online planning mode. (Firmware >= v1.11.0)
        '''
        request = SetInt16.Request()
        request.data = mode  # 설정할 모드

        # 서비스 호출
        future = self.service_clients['set_mode'].call_async(request)

        # 비동기 호출의 결과를 기다림
        while rclpy.ok() and not future.done():
            # 여기서는 별도로 spin_once가 이미 다른 스레드에서 처리되므로
            # 추가적으로 spin_once(self)를 호출하지 않습니다.
            time.sleep(RobotParam.spin_time)

        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Mode set to: {mode}, Response: {response.ret}, Message: {response.message}')
        else:
            self.get_logger().error('Failed to call service /xarm/set_mode')


    def call_elevation_command(self,meter):
        request=ElevationCommand.Request()
        request.move=meter
        request.until_complete=True

        future = self.service_clients['elevation_command'].call_async(request)

        # 비동기 호출의 결과를 기다림
        while rclpy.ok() and not future.done():
            # 여기서는 별도로 spin_once가 이미 다른 스레드에서 처리되므로
            # 추가적으로 spin_once(self)를 호출하지 않습니다.
            time.sleep(RobotParam.spin_time)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Servo Move Response: {response.successed}')
        else:
            self.get_logger().error('Failed to call service /elevation/command')

    def call_gripper_command(self,pose,force):
        request=GripperCommand.Request()
        request.pose=pose
        request.force=force

        future = self.service_clients['gripper_command'].call_async(request)


    def get_elevation_pose(self):
        cur_lift_position=copy.deepcopy(self.state.lm_current_pose)
        return cur_lift_position
