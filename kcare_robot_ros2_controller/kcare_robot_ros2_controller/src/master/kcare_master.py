import rclpy
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import ReliabilityPolicy, QoSProfile

from sensor_msgs.msg import Image,CameraInfo
from xarm_msgs.msg import RobotMsg
from xarm_msgs.srv import MoveJoint, MoveCartesian, SetInt16, SetInt16ById, Call
from kcare_robot_ros2_controller_msgs.msg import HeadCommand, HeadState, LMCommand, LMState
from kcare_robot_ros2_controller_msgs.srv import ElevationCommand, GripperCommand
from rosinterfaces.action import SendStringData as SendData

from  pyconnect.utils import str2dict, data_info, dict2str

from femto2handCalib import calibration
import copy

import time, threading

class KcareMaster(Node):
    def __init__(self):
        super().__init__('kcare_master')

        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()


        self.action_server = {
            'action_approach' : ActionServer(self,SendData,'/approach',lambda goal_handle: self.action_task_callback(goal_handle, 'approach'),callback_group=self.group1),
            'action_pick' : ActionServer(self,SendData,'/pick',lambda goal_handle: self.action_task_callback(goal_handle, 'pick'),callback_group=self.group1),
        }

        #self.kcaremaster_action_server = ActionServer(self,SendStringData,'/approach',self.task_approach_callback,callback_group=self.group1)

        self.service_clients = {
            'set_servo_angle': self.create_client(MoveJoint, '/xarm/set_servo_angle'),
            'set_servo_move': self.create_client(MoveCartesian, '/xarm/set_position'),
            'motion_enable': self.create_client(SetInt16ById, '/xarm/motion_enable'),
            'set_mode': self.create_client(SetInt16, '/xarm/set_mode'),
            'set_state': self.create_client(SetInt16, '/xarm/set_state'),
            'clean_error': self.create_client(Call, '/xarm/clean_error'),
            'elevation_command': self.create_client(ElevationCommand,'/elevation/set_position'),
            'gripper_command': self.create_client(GripperCommand, '/gripper/command')
        }
        
        self.topic_subscriber = {
            'robot_pose' : self.create_subscription(RobotMsg,'/xarm/robot_states', self.rb_pose_callback, 10,callback_group=self.group2),
            'lm_state' : self.create_subscription(LMState, '/elevation/state', self.lm_state_callback, 10,callback_group=self.group2),
            'head_state' : self.create_subscription(HeadState, '/head/state', self.head_state_callback, 10,callback_group=self.group2),
        }
        
        
        self.topic_publisher = {
            'head_command' : self.create_publisher(HeadCommand,'/head/command',10),
        }


        self.lm_current_pose=0.0
        self.pose=[0.0,0.0,0.0,0.0,0.0,0.0]
        self.head_state=[0.0,0.0]
        self.femto_calib=calibration()

        self.topic_ready = {key: False for key in self.topic_subscriber.keys()}

        # 서비스 준비 대기
        self.wait_for_services()
        # 토픽 준비 대기
        self.wait_for_topics()

        

        self.get_logger().info("✅ 모든 서비스 및 토픽이 준비 완료됨. 노드 시작!")




    def wait_for_services(self):
        """
        모든 서비스가 준비될 때까지 대기
        """
        for name, client in self.service_clients.items():
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(f"⏳ 서비스 {name} (/xarm/{name}) 준비 중...")
        self.get_logger().info("✅ 모든 서비스 연결 완료!")

    def wait_for_topics(self):
        """
        모든 토픽이 구독될 때까지 대기
        """
        self.get_logger().info("⏳ 모든 토픽의 데이터가 도착할 때까지 대기 중...")

        while not all(self.topic_ready.values()):
            rclpy.spin_once(self, timeout_sec=1.0)  # 노드 실행을 유지하면서 콜백 실행

        self.get_logger().info("✅ 모든 토픽의 데이터 수신 완료!")


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
        
        # 비동기 호출의 결과를 기다림
        while rclpy.ok() and not future.done():
            # 여기서는 별도로 spin_once가 이미 다른 스레드에서 처리되므로
            # 추가적으로 spin_once(self)를 호출하지 않습니다.
            time.sleep(0.1)

        if future.result() is not None:
            response = future.result()
            #self.get_logger().info(f'Servo Joint Response: {response.ret}, Message: {response.message}')
        else:
            self.get_logger().error('Failed to call service /xarm/set_servo_angle')


    def call_set_servo_move(self, pose, relative=False,mvtype=0):
        # 홈 위치로 서보 각도 설정 서비스 호출 예시
        request = MoveCartesian.Request()
        # 홈 위치 각도 설정
        request.pose = pose 
        request.speed = 50.0  # 속도 설정
        request.acc = 500.0    # 가속도 설정
        request.mvtime = 0.0    # 이동 시간 설정
        request.wait = True    # 완료 대기 설정
        request.relative = relative
        request.motion_type = mvtype

        # 서비스 호출
        future = self.service_clients['set_servo_move'].call_async(request)
        
        # 비동기 호출의 결과를 기다림
        # 비동기 호출의 결과를 기다림
        while rclpy.ok() and not future.done():
            # 여기서는 별도로 spin_once가 이미 다른 스레드에서 처리되므로
            # 추가적으로 spin_once(self)를 호출하지 않습니다.
            time.sleep(0.1)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Servo Move Response: {response.ret}, Message: {response.message}')
        else:
            self.get_logger().error('Failed to call service /xarm/set_position')
        

    def rb_pose_callback(self, msg):
        if not self.topic_ready['robot_pose']:
            self.get_logger().info("✅ /xarm/robot_states 토픽 준비됨.")
        self.topic_ready['robot_pose'] = True

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

    def call_elevation_command(self,meter):
        request=ElevationCommand.Request()
        request.move=meter
        request.until_complete=True

        future = self.service_clients['elevation_command'].call_async(request)

        # 비동기 호출의 결과를 기다림
        while rclpy.ok() and not future.done():
            # 여기서는 별도로 spin_once가 이미 다른 스레드에서 처리되므로
            # 추가적으로 spin_once(self)를 호출하지 않습니다.
            time.sleep(0.1)

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



    # def elevation_command(self,meter):
    #     # Input value range 0.195 0.935 
    #     lm_msg=LMCommand()
    #     lm_msg.cmd_type="abs"
    #     lm_msg.move=meter
    #     self.topic_publisher['lm_command'].publish(lm_msg)
    #     self.elevation_complete(meter)


    # def elevation_complete(self,target_meter):
    #     while True:
    #         driff =abs(target_meter-self.lm_current_pose)
    #         #self.get_logger().info(f'LM target: {target_meter}, LM state: {self.lm_current_pose}, LM driff: {driff}')

    #         if driff<0.001:
    #             break
    #         time.sleep(0.1)
    #     #self.get_logger().info(f'Elevation Complete Check state: {self.lm_state}')
    #     self.get_logger().info(f'Elevation Movement Complete')


    def lm_state_callback(self, msg):
        if not self.topic_ready['lm_state']:
            self.get_logger().info("✅ /elevation/state 토픽 준비됨.")
        self.topic_ready['lm_state'] = True
        self.lm_state=msg.state
        #self.get_logger().info(f'Elevation Callback Check state: {self.lm_state}')
        self.lm_current_pose=msg.current_position


    def head_command(self,rz,ry):
        head_msg=HeadCommand()
        head_msg.control_type="position"
        head_msg.rz=rz
        head_msg.ry=ry
        self.topic_publisher['head_command'].publish(head_msg)


    def head_state_callback(self, msg):
        if not self.topic_ready['head_state']:
            self.get_logger().info("✅ /head/state 토픽 준비됨.")
        self.topic_ready['head_state'] = True
        self.head_state=[msg.current_rz,msg.current_ry]


    def action_task_callback(self,goal_handle, action_name: str):
        #self.get_logger().info('Received action')

        rev_data = str2dict(goal_handle.request.data_goal)
        feedback = SendData.Feedback()


        if action_name == 'approach':
            #self.get_logger().info('Received action approach')
            #self.debug_approach(rev_data)
            self.task_approach(rev_data)
        elif action_name == 'pick':
            self.get_logger().info('Received action pick')
            self.task_pick()

        feedback.status = "Processing complete"
        goal_handle.publish_feedback(feedback)

        goal_handle.succeed()


        result = SendData.Result()
        result.data_result = dict2str(rev_data)
        #self.get_logger().info('Goal completed and data sent back')
        return result


    def debug_approach(self,rev_data):
        self.call_elevation_command(0.2)
        self.call_set_servo_angle([1.57,0.0,0.0,0.0,0.0,-1.57,0.0])
        
        img_pose=rev_data['pose']

        mid_x = (img_pose[0]+img_pose[2])/2
        mid_y = (img_pose[1]+img_pose[3])/2
        obj_depth = img_pose[4]

        cur_lift_position=copy.deepcopy(self.lm_current_pose)
        cur_arm_robot_pose=copy.deepcopy(self.pose)
        cur_angle_ry=copy.deepcopy(self.head_state[1])
        cur_angle_rz=copy.deepcopy(self.head_state[0])
        
        
        self.trans_lift_position,self.trans_armrobot_XYZ=self.femto_calib.convert_femto_to_arm(mid_x,mid_y,obj_depth,#[x,y,z] mm 
                                                                                cur_lift_position, # scalar (meter)
                                                                                cur_arm_robot_pose, # x y z r p y
                                                                                cur_angle_ry,# scalar
                                                                                cur_angle_rz# scalar
                                                                                )

        self.trans_x,self.trans_y,self.trans_z=self.trans_armrobot_XYZ
        self.get_logger().info(f'IMG Pose : {img_pose}, Lift : {self.trans_lift_position},Trans X: {self.trans_x}, Trans Y: {self.trans_y}, Trans Z: {self.trans_z}')
        

        elev_offset = 0.0


    def task_approach(self,rev_data):
        img_pose=rev_data['pose']

        self.call_gripper_command(1000,50)
        self.call_elevation_command(0.2)
        self.call_set_servo_angle([1.57,0.0,0.0,0.0,0.0,-1.57,0.0])

        mid_x = (img_pose[0]+img_pose[2])/2
        mid_y = (img_pose[1]+img_pose[3])/2
        obj_depth = img_pose[4]

        cur_lift_position=copy.deepcopy(self.lm_current_pose)
        cur_arm_robot_pose=copy.deepcopy(self.pose)
        cur_angle_ry=copy.deepcopy(self.head_state[1])
        cur_angle_rz=copy.deepcopy(self.head_state[0])
        
        
        self.trans_lift_position,self.trans_armrobot_XYZ=self.femto_calib.convert_femto_to_arm(mid_x,mid_y,obj_depth,#[x,y,z] mm 
                                                                                cur_lift_position, # scalar (meter)
                                                                                cur_arm_robot_pose, # x y z r p y
                                                                                cur_angle_ry,# scalar
                                                                                cur_angle_rz# scalar
                                                                                )


        self.trans_x,self.trans_y,self.trans_z=self.trans_armrobot_XYZ
        self.get_logger().info(f'Lift : {self.trans_lift_position},Trans X: {self.trans_x}, Trans Y: {self.trans_y}, Trans Z: {self.trans_z}')
        

        elev_offset = 0.0
        transy_offset = 40.0
        self.call_elevation_command(self.trans_lift_position+elev_offset)
        self.call_set_servo_move([0.0,self.trans_y-transy_offset,0.0,0.0,0.0,0.0],relative=True)
        self.get_logger().info(f'Calibration Movement Complete')
        #self.call_elevation_command(0.2)

    def task_pick(self):
        #self.call_gripper_command(1000,50)
        transx_offset=200.0
        self.call_set_servo_move([self.trans_x-transx_offset,0.0,0.0,0.0,0.0,0.0],relative=True)
        self.call_set_servo_move([-(self.trans_x-transx_offset)+50.0,0.0,0.0,0.0,0.0,0.0],relative=True)
        self.call_set_servo_angle([1.57,0.0,0.0,0.0,0.0,-1.57,0.0])
        self.call_elevation_command(0.2)




    def spin_once_in_thread(self):
        # spin_once를 별도의 스레드에서 실행하여 토픽 콜백을 처리
        while rclpy.ok():
            rclpy.spin_once(self)
            time.sleep(0.05)  # 콜백을 주기적으로 처리할 수 있도록 약간의 대기 시간 추가
            self.get_logger().info("Spinning")



    def init_robot(self):
        self.call_gripper_command(1000,50)

        self.call_motion_enable(8, 1)
        self.call_set_mode(0)
        self.call_set_state(0)

        self.call_elevation_command(0.2)
        self.call_set_servo_angle([1.57,0.0,0.0,0.0,0.0,0.0,0.0])

        self.head_command(0.0,15.0)


def main(args=None):
    rclpy.init(args=args)
    master_node = KcareMaster()
    
    #spin_thread = threading.Thread(target=master_node.spin_once_in_thread)
    #spin_thread.daemon = True  # 메인 프로그램 종료 시 스레드도 종료되도록 설정
    #spin_thread.start()
    
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(master_node)


    try:
        master_node.get_logger().info("✅ Action Server is running...")

        init_thread = threading.Thread(target=master_node.init_robot)
        init_thread.daemon = True
        init_thread.start()

        executor.spin()  # ✅ 멀티스레드 실행
    except KeyboardInterrupt:
        pass
    finally:
        init_thread.join()
        master_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()