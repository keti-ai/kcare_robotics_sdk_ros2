import rclpy
from rclpy.node import Node

from kcare_robot_ros2_controller.src.master.kcare_utils import RobotUtils

class KcareMaster(Node):
    def __init__(self):
        super().__init__('kcare_master')

        rbutils= RobotUtils()

        rbutils.xarm_init()



def main(args=None):
    rclpy.init(args=args)
    master_node = KcareMaster()

    try:
        master_node.get_logger().info("✅ Action Server is running...")
        rclpy.spin(master_node)
    except KeyboardInterrupt:
        pass
    finally:
        master_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()