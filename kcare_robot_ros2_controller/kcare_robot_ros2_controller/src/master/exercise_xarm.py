from rclpy.executors import MultiThreadedExecutor
from kcare_robot_ros2_controller.src.master.kcare_utils import *

class ExerciseNode(Node):
    def __init__(self):
        super().__init__('exercise')

        self.rbutils=RobotUtils(self)

    def init_motion(self):
        self.rbutils.call_motion_enable(8, 1)
        self.rbutils.call_set_mode(0)
        self.rbutils.call_set_state(0)
        self.rbutils.call_set_servo_angle(RobotParam.arm_home)
        self.rbutils.call_set_servo_angle(RobotParam.arm_ready)
        self.rbutils.call_set_relative_robot_pose(dx=30.0)


def main(args=None):
    rclpy.init(args=args)
    node = ExerciseNode()
    
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    executor.create_task(node.init_motion)
    try:
        node.get_logger().info("Node is running...")
        executor.spin()  
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()