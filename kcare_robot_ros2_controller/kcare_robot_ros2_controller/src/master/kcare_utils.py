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
    angle: list = [math.radians(90.0),0.0,0.0,0.0,0.0,0.0,0.0] # 로봇 현재 조인트값 저장 변수
    pose: list = [206.111, 6.175, 118.9, 3.14, 0.0, 0.0]    #로봇 현재 베이스좌표값 저장 변수
    rb_state: int = 0   # 로봇 스테이트값 저장변수
    rb_mode: int = 0    # 로봇 모드값 저장변수
    rb_error: int = 0   # 로봇 에러상태 저장변수
    lm_pose: float = 0.2    # 리프트 모듈 높이 저장 변수
    head_state: float = [0.0, 0.0]  # 팬틸트 각도 저장 변수

class RobotParam:
    spin_time: float = 0.05     # ROS2 루프문 대기시간
    elev_home: float = 0.2      # 리프트 홈위치
    arm_home: list = [math.radians(90.0),0.0,0.0,0.0,0.0,0.0,0.0]   # 조인트좌표계 로봇 홈자세
    arm_ready: list = [math.radians(90.0),math.radians(15.0),0.0,math.radians(15.0),0.0,math.radians(-90.0),0.0]    #조인트좌표계 로봇 준비자세
    arm_giving: list = [math.radians(130.0),math.radians(15.0),0.0,math.radians(15.0),0.0,math.radians(-90.0),0.0]
    j_arm_speed: float = 0.6    # 조인트좌표계 로봇 속도
    j_arm_accel: float = 10.0   # 조인트좌표계 로봇 가속도
    l_arm_speed: float = 200.0      # 베이스좌표계 로봇 속도
    l_arm_accel: float = 1000.0     # 베이스좌표계 로봇 가속도
    tool_radius: float = 50.0       # 로봇 툴 반지름
    tool_length: float = 230.0      # 로봇 툴팁 길이
    grip_open: int = 1000       # 그리퍼 오픈 상태 퍼센트
    grip_close: int = 0     #그리퍼 닫음 상태 퍼샌트
    grip_max_force: int = 100   # 그리퍼 최대힘
    grip_min_force: int = 50    # 그리퍼 최소힘



class RobotUtils:
    def __init__(self, node:Node):
        self.node = node    # 상위 노드에서 노드정보 상속
        self.rbstate=RobotState()
        
        self.topic_sub_group = MutuallyExclusiveCallbackGroup()

        SERVICE_CLIENTS ={
            'set_servo_angle' : ('/xarm/set_servo_angle',MoveJoint),
            'set_servo_move': ('/xarm/set_position', MoveCartesian),
            'motion_enable': ('/xarm/motion_enable', SetInt16ById),
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
        self.rbstate.rb_mode=msg.mode
        self.rbstate.rb_error=msg.err
        self.rbstate.rb_state=msg.state
        
        #self.node.get_logger().info(f"rb_pose_callback node.angle: {self.rbstate.angle}")
        # self.get_logger().info(f"rb_pose_callback node.pose: {self.state.pose}")

    def elevation_state_callback(self,msg):
        self.rbstate.lm_pose=msg.current_position
        #self.node.get_logger().info(f"lm_pose callback: {self.rbstate.lm_pose}")

    def head_state_callback(self,msg):
        self.rbstate.head_state=[msg.current_rz,msg.current_ry]
        #self.node.get_logger().info(f"head_pose callback: {self.rbstate.head_state}")

    def call_set_mode(self, mode=0):
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
    
    def call_set_state(self, state=0):
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

    def call_motion_enable(self, id=8, data=1):
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
        request.speed = RobotParam.j_arm_speed  # 속도 설정
        request.acc = RobotParam.j_arm_accel    # 가속도 설정
        request.mvtime = 0.0    # 이동 시간 설정
        request.wait = wait    # 완료 대기 설정
        # 서비스 호출
        future = self.service_clients['set_servo_angle'].call_async(request)
        
        if not wait:
            return

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
        request.speed = RobotParam.l_arm_speed  # 속도 설정
        request.acc = RobotParam.l_arm_accel    # 가속도 설정
        request.mvtime = 0.0    # 이동 시간 설정
        request.wait = wait    # 완료 대기 설정
        request.relative = relative
        request.motion_type = mvtype

        # 서비스 호출
        future = self.service_clients['set_servo_move'].call_async(request)
        
        if not wait:
            return

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
        
    def call_set_relative_robot_pose(self,dx=0.0,dy=0.0,dz=0.0,rdx=0.0,rdy=0.0,rdz=0.0,wait=True):
        target_pose=[dx,dy,dz,rdx,rdy,rdz]
        self.call_set_servo_move(target_pose,relative=True,wait=wait)

    def call_elevation_command(self,meter,wait=True):
        request=ElevationCommand.Request()
        request.move=meter
        request.until_complete=wait

        future = self.service_clients['elevation_command'].call_async(request)

        if not wait:
            return

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

    def call_gripper_command(self,pose,force=RobotParam.grip_min_force):
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

        if not wait:
            return
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
    
    def get_robot_state(self):
        err=copy.deepcopy(self.rbstate.rb_error)
        mode=copy.deepcopy(self.rbstate.rb_mode)
        state=copy.deepcopy(self.rbstate.rb_state)
        return {
            'err':err,
            'mode':mode,
            'state':state
        }

    def get_head_pose(self):
        cur_head_pose=copy.deepcopy(self.rbstate.head_state)
        return cur_head_pose
    
    