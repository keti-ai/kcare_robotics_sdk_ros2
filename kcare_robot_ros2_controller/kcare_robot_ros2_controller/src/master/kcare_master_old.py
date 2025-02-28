import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from kcare_robot_ros2_controller.src.master.kcare_utils import RobotUtils ,RobotParam
from rosinterfaces.action import SendStringData as ActSendData
from rosinterfaces.srv import SendStringData as SrvSendData

from pyconnect.utils import str2dict, data_info, dict2str

import time
import functools

class KcareMaster(Node):
    def __init__(self):
        super().__init__('kcare_master')

        self.rbutils=RobotUtils(self)
        self.act_callback_group = MutuallyExclusiveCallbackGroup()
        self.srv_callback_group = MutuallyExclusiveCallbackGroup()


        # ACTION_SERVER = {
        #     'action_approach' : '/approach',
        #     'action_pick' : '/pick',
        #     'action_place' : '/place',
        # }

        # self.action_server = {}
        # for action_tag, action_name in ACTION_SERVER.items():
        #     self.action_server[action_tag] = ActionServer(
        #         self,
        #         ActSendData,
        #         action_name,
        #         functools.partial(self.action_task_callback, action_name=action_tag),  # ✅ 안전한 콜백 바인딩
        #         callback_group=self.act_callback_group
        #     )
        #     self.get_logger().info(f"Action Server created: {action_tag} -> {action_name}")

        SERVICE_SERVER ={
            'move_head',
            'move',
            'approach',
            'pick',
            'place',
        }
        self.service_server = {}
        for service_name in SERVICE_SERVER:
            self.service_server[service_name] = self.create_service(
                SrvSendData,
                service_name,
                self.srv_callback,
                callback_group=self.srv_callback_group
            )
            self.get_logger().info(f"Service Server created: {service_name} -> {service_name}")



    # def action_task_callback(self,goal_handle, action_name: str):
    #     rev_data = str2dict(goal_handle.request.data_goal)
    #     feedback = ActSendData.Feedback()
    #     feedback.status = "Processing complete"


    #     goal_handle.publish_feedback(feedback)

    #     goal_handle.succeed()

    #     result = ActSendData.Result()
    #     result.data_result = dict2str(rev_data)
    #     self.get_logger().info('Goal completed and data sent back')
    #     return result

    def srv_callback(self,request,response):
        msg = request.req
        print(msg)

def main(args=None):
    rclpy.init(args=args)
    master_node = KcareMaster()
    
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(master_node)

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

