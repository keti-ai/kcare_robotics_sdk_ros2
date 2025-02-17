import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

import minimalmodbus
import concurrent.futures

from kcare_robot_ros2_controller_msgs.msg import GripperCommand, GripperState

class GripperControllerWrapper:
    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        
        # Initialize the minimalmodbus instrument
        self.instrument = minimalmodbus.Instrument(self.port, 1)  # 1 is the slave address
        self.instrument.serial.baudrate = self.baud
        self.instrument.serial.timeout = 1  # seconds

    def connect_grip(self):
        # minimalmodbus automatically handles serial port open/close, so no need for a separate connect method
        pass

    def disconnect_grip(self):
        self.instrument.serial.close()

    def gripper_initialize(self):
        """ 그리퍼 초기화 (Modbus Holding Register Address 0 -> Command 101) """
        self.instrument.write_register(0, 101)

        self.instrument.write_register(0, 213)  # Set Motor Speed Command
        self.instrument.write_register(1, 100)  # Set Speed to 100% (Max)

    def set_finger_position(self, position):
        """
        그리퍼 핑거 위치 설정
        position (int): 0 ~ 1000 (0: 닫힘, 1000: 열림)
        Modbus Holding Register Address 0 -> Command 104
        Modbus Holding Register Address 1 -> Target Position (0~1000)
        """
        if not (0 <= position <= 1000):
            raise ValueError("Finger position must be between 0 and 1000.")
        
        self.instrument.write_register(0, 104)  # Set Finger Position Command
        self.instrument.write_register(1, position)  # Target Position

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
