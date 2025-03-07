from rclpy.executors import MultiThreadedExecutor

from kcare_robot_ros2_controller.src.master.kcare_utils import *
from kcare_robot_ros2_controller_msgs.msg import RemoteCommand


import time


class KcareCtrlManager(Node):
    def __init__(self):
        super().__init__('kcare_control_manager')

        self.rbutils=RobotUtils(self)
        self.act_callback_group = MutuallyExclusiveCallbackGroup()
        self.srv_callback_group = MutuallyExclusiveCallbackGroup()
        self.sub_callback_group = MutuallyExclusiveCallbackGroup()

        TOPIC_SUBS = {
            'remote_controller': ('/kcare/remote_command', RemoteCommand,self.remote_control_callback),
        }

    def remote_control_callback(self,msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    master_node = KcareCtrlManager()
    
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(master_node)
    # 노드내 함수 비동기 실행.초기화 함수등 실행
    executor.create_task(master_node.rb_init)
    try:
        master_node.get_logger().info("✅ Master Server is running...")
        executor.spin()  # ✅ 멀티스레드 실행
    except KeyboardInterrupt:
        pass
    finally:
        master_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
