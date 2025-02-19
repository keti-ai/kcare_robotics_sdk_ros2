import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

from sensor_msgs.msg import Image,CameraInfo
from xarm_msgs.msg import RobotMsg
from xarm_msgs.srv import MoveJoint, MoveCartesian, SetInt16, SetInt16ById, Call
from kcare_robot_ros2_controller_msgs.msg import HeadCommand, HeadState, LMCommand, LMState
from rosinterfaces.action import SendStringData

from pyconnect.utils import str2dict

import time, threading

class KcareMaster(Node):
    def __init__(self):
        super().__init__('kcare_master')

        self.kcaremaster_action_server = ActionServer(self)

        self.service_clients = {
            'set_servo_angle': self.create_client(MoveJoint, '/xarm/set_servo_angle'),
            'set_servo_move': self.create_client(MoveCartesian, '/xarm/set_position'),
            'motion_enable': self.create_client(SetInt16ById, '/xarm/motion_enable'),
            'set_mode': self.create_client(SetInt16, '/xarm/set_mode'),
            'set_state': self.create_client(SetInt16, '/xarm/set_state'),
            'clean_error': self.create_client(Call, '/xarm/clean_error')
        }

        while not self.service_clients['set_servo_angle'].wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service /xarm/set_servo_angle not available, exiting...')
            return
        while not self.service_clients['set_servo_move'].wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service /xarm/set_servo_move not available, exiting...')
            return
        while not self.service_clients['motion_enable'].wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service /xarm/motion_enable not available, exiting...')
            return
        while not self.service_clients['set_mode'].wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service /xarm/set_mode not available, exiting...')
            return
        while not self.service_clients['set_state'].wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service /xarm/set_state not available, exiting...')
            return
        while not self.service_clients['clean_error'].wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service /xarm/clean_error not available, exiting...')
            return
        
        self.topic_subscriber = {
            'robot_pose' : self.create_subscription(RobotMsg,'/xarm/robot_states', self.rb_pose_callback, 10),
            'lm_state' : self.create_subscription(LMState, '/elevation/state', self.lm_state_callback, 10),
            'head_state' : self.create_subscription(HeadState, '/head/state', self.head_state_callback, 10),
        }
        
        
        self.topic_publisher = {
            'head_command' : self.create_publisher(HeadCommand,'/head/command',10),
            'lm_command' : self.create_publisher(LMCommand,'/elevation/command',10)
        }

        self.lm_state=False
        self.lm_current_pose=0.0


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
            time.sleep(0.1)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Mode set to: {mode}, Response: {response.ret}, Message: {response.message}')
        else:
            self.get_logger().error('Failed to call service /xarm/set_mode')
    
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
        while rclpy.ok() and not future.done():
            # 여기서는 별도로 spin_once가 이미 다른 스레드에서 처리되므로
            # 추가적으로 spin_once(self)를 호출하지 않습니다.
            time.sleep(0.1)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'State set to: {state}, Response: {response.ret}, Message: {response.message}')
        else:
            self.get_logger().error('Failed to call service /xarm/set_state')

    def call_motion_enable(self, id, data):
        # 로봇 토크 활성화
        request = SetInt16ById.Request()
        request.id = id
        request.data = data
        
        # 서비스 호출
        future = self.service_clients['motion_enable'].call_async(request)
        
        # 비동기 호출의 결과를 기다림
        while rclpy.ok() and not future.done():
            # 여기서는 별도로 spin_once가 이미 다른 스레드에서 처리되므로
            # 추가적으로 spin_once(self)를 호출하지 않습니다.
            time.sleep(0.1)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Motion Enabled Response: {response.ret}, Message: {response.message}')
        else:
            self.get_logger().error('Failed to call service /xarm/motion_enable')

    def call_clean_error(self):
        # 로봇 에러상태 클린
        request = Call.Request()
        
        future = self.service_clients['clean_error'].call_async(request)
        
        # 비동기 호출의 결과를 기다림
        while rclpy.ok() and not future.done():
            # 여기서는 별도로 spin_once가 이미 다른 스레드에서 처리되므로
            # 추가적으로 spin_once(self)를 호출하지 않습니다.
            time.sleep(0.1)
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Clean Error Response: {response.ret}, Message: {response.message}')
        else:
            self.get_logger().error('Failed to call service /xarm/clean_error')

    
    def call_set_servo_angle(self, angle):
        # 홈 위치로 서보 각도 설정 서비스 호출 예시
        request = MoveJoint.Request()
        
        # 홈 위치 각도 설정
        request.angles = angle 
        request.speed = 0.35  # 속도 설정
        request.acc = 10.0    # 가속도 설정
        request.mvtime = 0.0    # 이동 시간 설정
        request.wait = True    # 완료 대기 설정
        # 서비스 호출
        future = self.service_clients['set_servo_angle'].call_async(request)
        
        while rclpy.ok() and not future.done():
            # 여기서는 별도로 spin_once가 이미 다른 스레드에서 처리되므로
            # 추가적으로 spin_once(self)를 호출하지 않습니다.
            time.sleep(0.1)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Servo Joint Response: {response.ret}, Message: {response.message}')
        else:
            self.get_logger().error('Failed to call service /xarm/set_servo_angle')

    def call_set_servo_move(self, pose, relative=False):
        # 홈 위치로 서보 각도 설정 서비스 호출 예시
        request = MoveCartesian.Request()
        # 홈 위치 각도 설정
        request.pose = pose 
        request.speed = 50.0  # 속도 설정
        request.acc = 500.0    # 가속도 설정
        request.mvtime = 0.0    # 이동 시간 설정
        request.wait = True    # 완료 대기 설정
        request.relative = relative

        # 서비스 호출
        future = self.service_clients['set_servo_move'].call_async(request)
        
        # 비동기 호출의 결과를 기다림
        while rclpy.ok() and not future.done():
            # 여기서는 별도로 spin_once가 이미 다른 스레드에서 처리되므로
            time.sleep(0.1)
            

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Servo Move Response: {response.ret}, Message: {response.message}')
        else:
            self.get_logger().error('Failed to call service /xarm/set_position')

        

    def rb_pose_callback(self, msg):
        # 로봇팔 상태, 조인트 및 좌표 토픽 콜백  
        # self.get_logger().info(f'Robot State: "%s"' % str(msg.state))
        # self.get_logger().info(f'Robot Mode: "%s"' % str(msg.mode))
        # self.get_logger().info(f'Robot Error: "%s"' % str(msg.err))
        # self.get_logger().info(f'Robot Angles: "%s"' % str(msg.angle))
        # self.get_logger().info(f'Robot Pose: "%s"' % str(msg.pose))
        
        #TODO 조인트 및 포즈값 인덱스
        self.angle=msg.angle
        self.pose=msg.pose
        
        # print(f"rb_pose_callback node.angle: {self.angle}")
        # print(f"rb_pose_callback node.pose: {self.pose}")
        '''
        Log example
        [INFO] [1728973900.416814305] [kcare_calibrate]: Robot State: "5"
        [INFO] [1728973900.417027677] [kcare_calibrate]: Robot Mode: "0"
        [INFO] [1728973900.417180544] [kcare_calibrate]: Robot Error: "0"
        [INFO] [1728973900.417345285] [kcare_calibrate]: Robot Angles: "array('f', [1.5707963705062866, 6.938893903907228e-18, 2.168404344971009e-19, 6.938893903907228e-18, 2.168404344971009e-19, 3.469446951953614e-18, -5.421010862427522e-20])"
        [INFO] [1728973900.417735221] [kcare_calibrate]: Robot Pose: "[2.06111465e+02 6.17586040e+00 1.18902054e+02 3.13953233e+00 2.71484512e-03 5.84477501e-04]"
        '''

    def elevation_command(self,meter):
        # Input value range 0.195 0.935 
        lm_msg=LMCommand()
        lm_msg.cmd_type="abs"
        lm_msg.move=meter
        self.topic_publisher['lm_command'].publish(lm_msg)
        self.elevation_complete(meter)

    def elevation_complete(self,target_meter):
        while True:
            driff =abs(target_meter-self.lm_current_pose)
            #self.get_logger().info(f'LM target: {target_meter}, LM state: {self.lm_current_pose}, LM driff: {driff}')

            if driff<0.001:
                break
            time.sleep(0.1)
        #self.get_logger().info(f'Elevation Complete Check state: {self.lm_state}')
        self.get_logger().info(f'Elevation Movement Complete')

    def lm_state_callback(self, msg):
        self.lm_state=msg.state
        #self.get_logger().info(f'Elevation Callback Check state: {self.lm_state}')
        self.lm_current_pose=msg.current_position

    def head_state_callback(self, msg):
        # Arm 카메라 RGB 이미지 콜백
        self.head_state=[msg.current_rz,msg.current_ry]


    def spin_once_in_thread(self):
        # spin_once를 별도의 스레드에서 실행하여 토픽 콜백을 처리
        while rclpy.ok():
            rclpy.spin_once(self)
            time.sleep(0.05)  # 콜백을 주기적으로 처리할 수 있도록 약간의 대기 시간 추가


def main(args=None):
    rclpy.init(args=args)
    master_node = KcareMaster()
    
    spin_thread = threading.Thread(target=master_node.spin_once_in_thread)
    spin_thread.daemon = True  # 메인 프로그램 종료 시 스레드도 종료되도록 설정
    spin_thread.start()
    
    master_node.call_motion_enable(8, 1)
    master_node.call_set_mode(0)
    master_node.call_set_state(0)

    master_node.elevation_command(0.195)
    master_node.call_set_servo_angle([[1.57,0.0,0.0,0.0,0.0,-1.57,0.0]])


    while rclpy.ok():
        time.sleep(0.1)

    master_node.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()