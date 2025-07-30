import rclpy
from rclpy.node import Node

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from kcare_robot_ros2_controller.src.head.dxl_utils.dxl_prop import Dynamixel
from kcare_robot_ros2_controller_msgs.msg import HeadCommand, HeadState, HeadGoHomeRequest
from kcare_robot_ros2_controller_msgs.srv import HeadPoseCommand

import time
class HeadControlNode(Node):
    def __init__(self):
        super().__init__('head_control_node')
        
        self.baudrate = 57600
        self.protocol = 2.0
        self.device_name = '/dev/ttyHead'
        self.device_id = [1, 2]
        self.device_home = [2048, 2048]                 # step
        self.device_limit = [[500, 3600], [1700, 2700]] # step
        self.device_speed = [40, 40] # step
        self.device_deg_offset = [180, 180]
        
        self.target_rz=0
        self.target_ry=0

        self.cur_rz=0.0
        self.cur_ry=0.0

        self.dxl = None
        self.set()
        self.init()

        self.group1=MutuallyExclusiveCallbackGroup()

        self.subscriber_move = self.create_subscription(HeadCommand,
                                                        'head/command',
                                                        self.topic_callback_move,
                                                        10,callback_group=self.group1)
         
        self.subscriber_go_home = self.create_subscription(HeadGoHomeRequest,
                                                           'head/go_home',
                                                           self.topic_callback_go_home,
                                                           10,callback_group=self.group1)
        
        self.state_publisher = self.create_publisher(HeadState,
                                                     'head/state',
                                                     10)

        self.service_client = self.create_service(HeadPoseCommand,'head/pose_command',self.service_callback_pose, callback_group=self.group1)


        self.timer_state = self.create_timer(0.1, self.timer_callback_state)

    def set(self):
        self.dxl = Dynamixel(self.get_logger())
        self.dxl.set(baudrate=self.baudrate, 
                     protocol=self.protocol,
                     device_name=self.device_name,
                     device_ids=self.device_id,
                     device_home=self.device_home,
                     device_limit=self.device_limit,
                     device_speed=self.device_speed,
                     device_offset=self.device_deg_offset)
        
    def init(self):
        self.dxl.init()
        self.dxl.go_home()

    def topic_callback_move(self, msg):
        self.dxl.control_mode = msg.control_type
        
        if msg.control_type == 'position':
            rz = int(msg.rz)
            ry = int(msg.ry)
            
            
            self.update_target_pose(rz, ry)
            # self.get_logger().info(f"{self.target_rz}, {self.target_ry}")    
            self.dxl.position_control(self.target_rz, self.target_ry)

        elif msg.control_type == 'velocity':
            rz = int(msg.rz)
            ry = int(msg.ry)

            self.update_target_pose(rz, ry)
            self.dxl.velocity_control(self.target_rz, self.target_ry)
        

    def topic_callback_go_home(self, msg):
        if msg:
            self.dxl.go_home()
            self.reset_target_pose()


    def service_callback_pose(self, request, response):
        """
        ✅ 요청된 rz, ry 값으로 모터를 이동시키고, 목표 위치에 도달하면 응답 반환
        """
        self.dxl.control_mode = 'position'

        target_rz = -int(request.rz)
        target_ry = -int(request.ry)
        
        self.reset_target_pose()
        self.update_target_pose(target_rz, target_ry)
        
        # self.get_logger().info(f"{self.target_rz}, {self.target_ry}")   
        self.dxl.position_control(self.target_rz, self.target_ry)

        # 비동기 완료 처리
        if not request.wait:
            response.successed=True
            return response

        # ✅ 목표 위치 도달 감지 (허용 오차 범위 설정)
        tolerance = 2.5  # 2도 이내면 도달한 것으로 간주 (조정 가능)
        timeout = 2.0  # 최대 대기 시간 (초)
        start_time = time.time()

        while time.time() - start_time < timeout:
            # ✅ 목표 위치 도달 여부 확인
            if abs(self.cur_rz + self.target_rz) <= tolerance and abs(self.cur_ry + self.target_ry) <= tolerance:
                response.successed = True
                return response  # ✅ 도달 시 바로 응답 반환

            time.sleep(0.1)  # 100ms 대기 후 다시 체크

        # ✅ 시간 초과 시 실패 처리
        response.successed = False
        return response
    
    def reset_target_pose(self):
        self.target_ry = 0
        self.target_rz = 0

    def update_target_pose(self,target_rz,target_ry):
        self.target_rz = max(min(self.target_rz + target_rz, 60), -60)
        self.target_ry = max(min(self.target_ry + target_ry, 40), -35)

    def update_current_pose(self):
        cur_rz = self.dxl.get_pose(1)
        cur_ry = self.dxl.get_pose(2)
        return cur_rz, cur_ry

    def timer_callback_state(self):
        self.cur_rz, self.cur_ry = self.update_current_pose()

        state_msg = HeadState()
        state_msg.enable = self.dxl.dxl_enable
        state_msg.control_type = self.dxl.control_mode
        state_msg.current_rz = self.cur_rz
        state_msg.current_ry = self.cur_ry
        state_msg.speed_rz = float(self.dxl.speed(1))
        state_msg.speed_ry = float(self.dxl.speed(2))
        
        self.state_publisher.publish(state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HeadControlNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
