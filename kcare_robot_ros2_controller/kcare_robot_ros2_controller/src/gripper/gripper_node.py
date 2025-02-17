import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

from pymodbus.client import ModbusSerialClient
import concurrent.futures

import time

from kcare_robot_ros2_controller_msgs.msg import GripperCommand, GripperState

class GripperControllerWrapper:
    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        
        # Initialize the pymodbus
        self.client = ModbusSerialClient(self.port, baudrate=self.baud)


    def connect_grip(self):
        # minimalmodbus automatically handles serial port open/close, so no need for a separate connect method
        self.client.connect()

    def disconnect_grip(self):
        self.client.close()

    def gripper_initialize(self):
        self.client.write_register(0,101,slave=1)

    def set_finger_position(self, position):
        if not (0 <= position <= 1000):
            print("⚠️ 유효한 값이 아닙니다. (0 ~ 1000)")
            return
        
        print(f"🔄 Gripper Finger Position 설정: {position}...")
        self.client.write_register(0, 104, slave=1)
        self.client.write_register(1, position, slave=1)  # 목표 값 설정
        print(f"✅ Finger Position {position} 설정 완료!")


class GripperNode(Node):
    def __init__(self):
        super().__init__('gripper_subscriber')
        self.gripper_client = GripperControllerWrapper("/dev/ttyGripper", 115200)
        # self.gripper_client = GripperControllerWrapper("/dev/ttyACM2", 115200)
        self.gripper_client.connect_grip()
        self.gripper_client.gripper_initialize()

        self.subscriber = self.create_subscription(
            GripperCommand,
            'gripper/command',
            self.topic_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

    def topic_callback(self, msg):
        position = msg.pose  # GripperCommand 메시지에서 pose 값 사용 (0~1000 범위)
        
        with concurrent.futures.ThreadPoolExecutor() as executor:
            executor.submit(self.gripper_client.set_finger_position, position)

def main(args=None):
    rclpy.init(args=args)

    subscriber_ = GripperNode()
    rclpy.spin(subscriber_)

    subscriber_.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
