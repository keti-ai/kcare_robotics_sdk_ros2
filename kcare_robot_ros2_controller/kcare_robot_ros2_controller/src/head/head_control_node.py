import time

import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from kcare_robot_ros2_controller.src.head.dxl_utils.dxl_prop import Dynamixel
from kcare_robot_ros2_controller_msgs.msg import HeadCommand, HeadState, HeadGoHomeRequest
from kcare_robot_ros2_controller_msgs.srv import HeadPoseCommand

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
        
        self.dxl = None
        self.set()
        self.init()

        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()

        self.subscriber_move = self.create_subscription(HeadCommand,
                                                        'head/command',
                                                        self.topic_callback_move,
                                                        10, callback_group=self.group1)
         
        self.subscriber_go_home = self.create_subscription(HeadGoHomeRequest,
                                                           'head/go_home',
                                                           self.topic_callback_go_home,
                                                           10, callback_group=self.group1)
        
        self.state_publisher = self.create_publisher(HeadState,
                                                     'head/state',
                                                     10)

        self.service_client = self.create_service(HeadPoseCommand,'head/pose_command',self.service_callback_pose, callback_group=self.group1)


        self.timer_state = self.create_timer(0.5, self.timer_callback_state, callback_group=self.group2)

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
        if msg.control_type == 'position':
            rz = -int(msg.rz)
            ry = -int(msg.ry)
            
            self.dxl.position_control(rz, ry)
            
        elif msg.control_type == 'velocity':
            rz = int(msg.rz)
            ry = int(msg.ry)

            self.dxl.velocity_control(rz, ry)

    def topic_callback_go_home(self, msg):
        if msg:
            self.dxl.go_home()
    
    def service_callback_pose(self, request, response):
        """
        ✅ 요청된 rz, ry 값으로 모터를 이동시키고, 목표 위치에 도달하면 응답 반환
        """
        target_rz = -int(request.rz)
        target_ry = -int(request.ry)

        # ✅ 모터 이동 명령 실행
        self.dxl.position_control(target_rz, target_ry)

        # ✅ 목표 위치 도달 감지 (허용 오차 범위 설정)
        tolerance = 2  # 2도 이내면 도달한 것으로 간주 (조정 가능)
        timeout = 5.0  # 최대 대기 시간 (초)
        start_time = time.time()

        while time.time() - start_time < timeout:
            # ✅ 현재 위치 확인
            current_rz = self.dxl.get_pose(1)
            current_ry = self.dxl.get_pose(2)

            # ✅ 목표 위치 도달 여부 확인
            if abs(current_rz - target_rz) <= tolerance and abs(current_ry + target_ry) <= tolerance:
                response.successed = True
                return response  # ✅ 도달 시 바로 응답 반환

            time.sleep(0.1)  # 100ms 대기 후 다시 체크

        # ✅ 시간 초과 시 실패 처리
        response.successed = False
        return response

    def timer_callback_state(self):
        state_msg = HeadState()
        state_msg.enable = self.dxl.dxl_enable
        state_msg.control_type = self.dxl.control_mode
        state_msg.current_rz = self.dxl.get_pose(1)
        state_msg.current_ry = self.dxl.get_pose(2)
        state_msg.speed_rz = float(self.dxl.speed(1))
        state_msg.speed_ry = float(self.dxl.speed(2))
        
        self.state_publisher.publish(state_msg)
    
def main(args=None):
    rclpy.init(args=args)
    node = HeadControlNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)

    try:
        node.get_logger().info("🚀 HeadControl running with MultiThreadedExecutor")
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()