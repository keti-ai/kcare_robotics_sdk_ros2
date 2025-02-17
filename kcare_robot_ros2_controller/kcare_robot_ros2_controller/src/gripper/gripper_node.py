import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

from pymodbus.client import ModbusSerialClient
import concurrent.futures

import time

from kcare_robot_ros2_controller_msgs.msg import GripperState
from kcare_robot_ros2_controller_msgs.srv import GripperCommand

class GripperControllerWrapper:
    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        
        # Initialize the pymodbus
        self.client = ModbusSerialClient(self.port, baudrate=self.baud)


    def connect_grip(self):

        self.client.connect()

    def disconnect_grip(self):
        self.client.close()

    def gripper_initialize(self):
        self.client.write_register(0,101,slave=1)

        self.client.write_register(0,213,slave=1)
        self.client.write_register(1,100,slave=1)

    def set_finger_position(self, position):
        self.client.write_register(0, 104, slave=1)
        self.client.write_register(1, position, slave=1)  # 목표 값 설정

    def set_motor_torque(self,ratio):
        self.client.write_register(0,212, slave=1)
        self.client.write_register(1,ratio,slave=1)


    def read_status(self):
        MOTOR_STATUS_REGISTERS = {
        "operation_mode": 0x0000,   # 모터 동작 모드
        "speed": 0x0001,            # 현재 속도
        "position": 0x0002,         # 현재 위치
        "torque": 0x0003,           # 현재 토크 값
        "temperature": 0x0004,      # 모터 온도
        "voltage": 0x0005,          # 공급 전압
        "current": 0x0006,          # 현재 전류
        "error_status": 0x0007      # 에러 상태
        }
        for name, address in MOTOR_STATUS_REGISTERS.items():
            try:
                result = self.client.read_holding_registers(address, count=1, slave=1)
                if result.isError():
                    print(f"⚠️ [{name}] 읽기 실패! (Address: {hex(address)})")
                else:
                    value = result.registers[0]
                    print(f"✅ {name}: {value} (Address: {hex(address)})")
            except Exception as e:
                print(f"❌ [{name}] 오류 발생: {e}")



class GripperNode(Node):
    def __init__(self):
        super().__init__('gripper_subscriber')
        self.gripper_client = GripperControllerWrapper("/dev/ttyGripper", 115200)
        # self.gripper_client = GripperControllerWrapper("/dev/ttyACM2", 115200)
        self.gripper_client.connect_grip()
        self.gripper_client.gripper_initialize()

        self.gripper_service= self.create_service(GripperCommand,'gripper/command',self.set_gripperpose_callback)

        self.publisher = self.create_publisher(GripperState,
                                               'gripper/state',
                                               10)

        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)
        

    def set_gripperpose_callback(self,request,response):
        self.get_logger().info(f"Gripper Service Call.")
        if (0 <= request.pose <= 1000 and 50<= request.force <= 100):
            self.gripper_client.set_motor_torque(request.force)
            self.gripper_client.set_finger_position(request.pose)
            self.get_logger().info(f"Gripper Work Fine. Pose : {request.pose}, Force : {request.force}.")
            response.successed=True
        else:
            self.get_logger().info(f"Gripper request out of range.")
            response.successed=False
        return response


    def timer_callback(self):
        self.gripper_client.read_status()


def main(args=None):
    rclpy.init(args=args)

    subscriber_ = GripperNode()
    rclpy.spin(subscriber_)

    subscriber_.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
