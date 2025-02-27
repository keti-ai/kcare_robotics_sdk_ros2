import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from kcare_robot_ros2_controller.src.master.kcare_utils import RobotUtils
from rosinterfaces.action import SendStringData as SendData

from pyconnect.utils import str2dict, data_info, dict2str
import time

class KcareMaster(Node):
    def __init__(self):
        super().__init__('kcare_master')

        self.rbutils= RobotUtils()

        self.action_server = {
            'action_approach' : ActionServer(self,SendData,'/approach',lambda goal_handle: self.action_task_callback(goal_handle, 'approach'),callback_group=self.rbutils.act_callback_group)
        }

    def action_task_callback(self,goal_handle, action_name: str):
        rev_data = str2dict(goal_handle.request.data_goal)
        feedback = SendData.Feedback()
        feedback.status = "Processing complete"

        goal_handle.publish_feedback(feedback)

        goal_handle.succeed()


        result = SendData.Result()
        result.data_result = dict2str(rev_data)
        self.get_logger().info('Goal completed and data sent back')
        return result



def main(args=None):
    rclpy.init(args=args)
    master_node = KcareMaster()
    
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(master_node)
    executor.add_node(master_node.rbutils)


    try:
        master_node.get_logger().info("✅ Action Server is running...")

        executor.spin()  # ✅ 멀티스레드 실행
    except KeyboardInterrupt:
        pass
    finally:
        master_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

