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

    def initializing(self):
        # Write register 0x0100 with value 0xA5 on slave address 1
        self.instrument.write_register(0x0100, 0xA5)

    def gripper_pose(self, pose):
        # Write register 0x0103 with the given pose value
        self.instrument.write_register(0x0103, pose)


class GripperNode(Node):
    def __init__(self):
        super().__init__('gripper_subscriber')
        self.gripper_client = GripperControllerWrapper("/dev/ttyGripper", 115200)
        # self.gripper_client = GripperControllerWrapper("/dev/ttyACM2", 115200)
        self.gripper_client.connect_grip()
        self.gripper_client.initializing()

        self.subscriber = self.create_subscription(
            GripperCommand,
            'gripper/command',
            self.topic_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

    def topic_callback(self, msg):
        pose = msg.pose
        # self.get_logger().info(pose)
        with concurrent.futures.ThreadPoolExecutor() as executor:
            future = executor.submit(self.gripper_client.gripper_pose, pose)


def main(args=None):
    rclpy.init(args=args)

    subscriber_ = GripperNode()
    rclpy.spin(subscriber_)

    subscriber_.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
