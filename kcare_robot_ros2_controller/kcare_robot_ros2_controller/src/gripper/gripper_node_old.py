import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

from pymodbus.client import ModbusSerialClient
from kcare_robot_ros2_controller_msgs.msg import GripperCommand, GripperState


class GripperControllerWrapper:
    def __init__(self, port, baud):
        self.port = port
        self.baud = baud

        self.client = ModbusSerialClient(self.port, baudrate=self.baud)

    def connect_grip(self):
        self.client.connect()

    def disconnect_grip(self):
        self.client.close()

    def initializing(self):
        self.client.write_register(0x0100, 0xA5, slave=1)

    def gripper_pose(self, pose):
        self.client.write_register(0x0103, pose)


class GripperNode(Node):
    def __init__(self):
        super().__init__('gripper_subscriber')
        self.gripper_client = GripperControllerWrapper("/dev/ttyGripper", 115200)
        self.gripper_client.connect_grip()
        self.gripper_client.initializing()

        self.subscriber = self.create_subscription(GripperCommand, 'gripper/command', self.topic_callback,
                                                   QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

    def topic_callback(self, msg):
        pose = msg.pose
        # self.get_logger().info(pose)
        self.gripper_client.gripper_pose(pose)


def main(args=None):
    rclpy.init(args=args)

    subscriber_ = GripperNode()
    rclpy.spin(subscriber_)

    subscriber_.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()