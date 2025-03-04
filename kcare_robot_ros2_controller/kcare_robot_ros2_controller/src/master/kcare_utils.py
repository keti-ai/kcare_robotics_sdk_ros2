import rclpy
import rclpy.executors
from rclpy.node import Node

from xarm_msgs.msg import RobotMsg
from xarm_msgs.srv import MoveJoint, MoveCartesian, SetInt16, SetInt16ById, Call
from kcare_robot_ros2_controller_msgs.msg import HeadState, LMState
from kcare_robot_ros2_controller_msgs.srv import ElevationCommand, GripperCommand, HeadPoseCommand

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


import time, math, copy

class RobotState:
    angle: list = [math.radians(90.0),0.0,0.0,0.0,0.0,0.0,0.0] 
    pose: list = [206.111, 6.175, 118.9, 3.14, 0.0, 0.0]
    lm_pose: float = 0.2
    head_state: float = [0.0, 0.0]

class RobotParam:
    spin_time: float = 0.05
    elev_home: float = 0.2
    arm_home: list = [math.radians(90.0),0.0,0.0,0.0,0.0,0.0,0.0]
    arm_ready: list = [math.radians(90.0),math.radians(15.0),0.0,math.radians(15.0),0.0,math.radians(-90.0),0.0]
    tool_radius: float = 50.0
    tool_length: float = 200.0
    grip_open: int = 1000
    grip_close: int = 0
    grip_max_force: int = 100
    grip_min_force: int = 50



class RobotUtils:
    def __init__(self, node:Node):
        self.node = node
        self.rbstate=RobotState()
        
        self.topic_sub_group = MutuallyExclusiveCallbackGroup()
        #self.act_callback_group = MutuallyExclusiveCallbackGroup()

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

        self.service_clients = {}
        for service_tag, (service_name, service_type) in SERVICE_CLIENTS.items():
            self.service_clients[service_tag] = self.node.create_client(service_type, service_name)
            self.node.get_logger().info(f"Service Client created: {service_tag} -> {service_name}")

        TOPIC_SUBS = {
            'robot_pose': ('/xarm/robot_states', RobotMsg,self.robot_pose_callback),
            'lm_state': ('/elevation/state', LMState,self.elevation_state_callback),
            'head_state': ('/head/state', HeadState,self.head_state_callback),
        }

        # 토픽 구독 설정
        self.topic_subs = {}
        for topic_tag, (topic_name, topic_type,topic_callback_fun) in TOPIC_SUBS.items():
            self.topic_subs[topic_tag] = self.node.create_subscription(topic_type,topic_name,topic_callback_fun,10,callback_group=self.topic_sub_group)
            self.node.get_logger().info(f"Subscriber created: {topic_tag} -> {topic_name} with {topic_callback_fun}")

    def robot_pose_callback(self, msg):
        # 로봇팔 상태, 조인트 및 좌표 토픽 콜백  
        # self.get_logger().info(f'Robot State: "%s"' % str(msg.state))
        # self.get_logger().info(f'Robot Mode: "%s"' % str(msg.mode))
        # self.get_logger().info(f'Robot Error: "%s"' % str(msg.err))
        # self.get_logger().info(f'Robot Angles: "%s"' % str(msg.angle))
        # self.get_logger().info(f'Robot Pose: "%s"' % str(msg.pose))
        
        #TODO 조인트 및 포즈값 인덱스
        self.rbstate.angle=msg.angle
        self.rbstate.pose=msg.pose
        
        #self.node.get_logger().info(f"rb_pose_callback node.angle: {self.rbstate.angle}")
        # self.get_logger().info(f"rb_pose_callback node.pose: {self.state.pose}")

    def elevation_state_callback(self,msg):
        self.rbstate.lm_pose=msg.current_position
        #self.node.get_logger().info(f"lm_pose callback: {self.rbstate.lm_pose}")

    def head_state_callback(self,msg):
        self.rbstate.head_state=[msg.current_rz,msg.current_ry]
        #self.node.get_logger().info(f"head_pose callback: {self.rbstate.head_state}")

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
        while not future.done():
            # 여기서는 별도로 spin_once가 이미 다른 스레드에서 처리되므로
            # 추가적으로 spin_once(self)를 호출하지 않습니다.
            time.sleep(RobotParam.spin_time)

        
        if future.result() is not None:
            response = future.result()
            self.node.get_logger().info(f'Mode set to: {mode}, Response: {response.ret}, Message: {response.message}')
        else:
            self.node.get_logger().error('Failed to call service /xarm/set_mode')
    
    def call_set_state(self, state):
        # 로봇 스테이트 관련 서비스 호출 예시
        '''
        state:
        0: motion state
        3: pause state
        4: stop state
        '''
        request = SetInt16.Request()
        request.data = state  # 설정할 상태

        # 서비스 호출
        future = self.service_clients['set_state'].call_async(request)
        
        # 비동기 호출의 결과를 기다림
        while not future.done():
            # 여기서는 별도로 spin_once가 이미 다른 스레드에서 처리되므로
            # 추가적으로 spin_once(self)를 호출하지 않습니다.
            time.sleep(RobotParam.spin_time)
        
        if future.result() is not None:
            response = future.result()
            self.node.get_logger().info(f'State set to: {state}, Response: {response.ret}, Message: {response.message}')
        else:
            self.node.get_logger().error('Failed to call service /xarm/set_state')

    def call_motion_enable(self, id, data):
        # 로봇 토크 활성화
        request = SetInt16ById.Request()
        request.id = id
        request.data = data
        
        # 서비스 호출
        future = self.service_clients['motion_enable'].call_async(request)

        # 비동기 호출의 결과를 기다림
        while not future.done():
            # 여기서는 별도로 spin_once가 이미 다른 스레드에서 처리되므로
            # 추가적으로 spin_once(self)를 호출하지 않습니다.
            time.sleep(RobotParam.spin_time)
        
        if future.result() is not None:
            response = future.result()
            self.node.get_logger().info(f'Motion Enabled Response: {response.ret}, Message: {response.message}')
        else:
            self.node.get_logger().error('Failed to call service /xarm/motion_enable')

    def call_clean_error(self):
        # 로봇 에러상태 클린
        request = Call.Request()
        
        future = self.service_clients['clean_error'].call_async(request)
        
        # 비동기 호출의 결과를 기다림
        while not future.done():
            # 여기서는 별도로 spin_once가 이미 다른 스레드에서 처리되므로
            # 추가적으로 spin_once(self)를 호출하지 않습니다.
            time.sleep(RobotParam.spin_time)

        if future.result() is not None:
            response = future.result()
            self.node.get_logger().info(f'Clean Error Response: {response.ret}, Message: {response.message}')
        else:
            self.node.get_logger().error('Failed to call service /xarm/clean_error')

    def call_set_servo_angle(self, angle,wait=True):
        # 홈 위치로 서보 각도 설정 서비스 호출 예시
        request = MoveJoint.Request()
        
        # 홈 위치 각도 설정
        request.angles = angle 
        request.speed = 0.7  # 속도 설정
        request.acc = 20.0    # 가속도 설정
        request.mvtime = 0.0    # 이동 시간 설정
        request.wait = wait    # 완료 대기 설정
        # 서비스 호출
        future = self.service_clients['set_servo_angle'].call_async(request)
        
        # 비동기 호출의 결과를 기다림
        while not future.done():
            # 여기서는 별도로 spin_once가 이미 다른 스레드에서 처리되므로
            # 추가적으로 spin_once(self)를 호출하지 않습니다.
            time.sleep(RobotParam.spin_time)

        if future.result() is not None:
            response = future.result()
            self.node.get_logger().info(f'Servo Joint Response: {response.ret}, Message: {response.message}')
        else:
            self.node.get_logger().error('Failed to call service /xarm/set_servo_angle')


    def call_set_servo_move(self, pose, relative=False,mvtype=0,wait=True):
        # 홈 위치로 서보 각도 설정 서비스 호출 예시
        request = MoveCartesian.Request()
        # 홈 위치 각도 설정
        request.pose = pose 
        request.speed = 100.0  # 속도 설정
        request.acc = 1000.0    # 가속도 설정
        request.mvtime = 0.0    # 이동 시간 설정
        request.wait = wait    # 완료 대기 설정
        request.relative = relative
        request.motion_type = mvtype

        # 서비스 호출
        future = self.service_clients['set_servo_move'].call_async(request)
        
        # 비동기 호출의 결과를 기다림
        while not future.done():
            # 여기서는 별도로 spin_once가 이미 다른 스레드에서 처리되므로
            # 추가적으로 spin_once(self)를 호출하지 않습니다.
            time.sleep(RobotParam.spin_time)

        if future.result() is not None:
            response = future.result()
            self.node.get_logger().info(f'Servo Move Response: {response.ret}, Message: {response.message}')
        else:
            self.node.get_logger().error('Failed to call service /xarm/set_position')
        
    def call_elevation_command(self,meter,wait=True):
            request=ElevationCommand.Request()
            request.move=meter
            request.until_complete=wait

            future = self.service_clients['elevation_command'].call_async(request)

            # 비동기 호출의 결과를 기다림
            while not future.done():
                # 여기서는 별도로 spin_once가 이미 다른 스레드에서 처리되므로
                # 추가적으로 spin_once(self)를 호출하지 않습니다.
                time.sleep(RobotParam.spin_time)

            if future.result() is not None:
                response = future.result()
                self.node.get_logger().info(f'Servo Move Response: {response.successed}')
            else:
                self.node.get_logger().error('Failed to call service /elevation/command')

    def call_gripper_command(self,pose,force):
        request=GripperCommand.Request()
        request.pose=pose
        request.force=force

        future = self.service_clients['gripper_command'].call_async(request)

        # 비동기 호출의 결과를 기다림
        while not future.done():
            # 여기서는 별도로 spin_once가 이미 다른 스레드에서 처리되므로
            # 추가적으로 spin_once(self)를 호출하지 않습니다.
            time.sleep(RobotParam.spin_time)

        if future.result() is not None:
            response = future.result()
            self.node.get_logger().info(f'Gripper Response: ok')
        else:
            self.node.get_logger().error('Failed to call service /gripper/command')

    def call_head_command(self,head_pose,wait=True):
        request=HeadPoseCommand.Request()
        request.rz = head_pose[0]
        request.ry = head_pose[1]
        request.wait = wait

        future = self.service_clients['head_command'].call_async(request)

        # 비동기 호출의 결과를 기다림
        while not future.done():
            # 여기서는 별도로 spin_once가 이미 다른 스레드에서 처리되므로
            # 추가적으로 spin_once(self)를 호출하지 않습니다.
            time.sleep(RobotParam.spin_time)

        if future.result() is not None:
            response = future.result()
            self.node.get_logger().info(f'Head Move : {response.successed}')
        else:
            self.node.get_logger().error('Failed to call service /gripper/command') 


    def get_elev_pose(self):
        cur_lift_position=copy.deepcopy(self.rbstate.lm_pose)
        return cur_lift_position

    def get_robot_pose(self):
        cur_arm_robot_pose=copy.deepcopy(self.rbstate.pose)
        cur_arm_robot_joint=copy.deepcopy(self.rbstate.angle)
        return {'pose':cur_arm_robot_pose,
                'joint':cur_arm_robot_joint}
    
    def get_head_pose(self):
        cur_head_pose=copy.deepcopy(self.rbstate.head_state)
        return cur_head_pose