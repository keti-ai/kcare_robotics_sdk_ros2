import time

import rclpy
from rclpy.node import Node

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from kcare_robot_ros2_controller.src.elevation.md_utils.MD485driver import MD485DriverWrapper

from kcare_robot_ros2_controller_msgs.msg import LMCommand, LMState
from kcare_robot_ros2_controller_msgs.srv import ElevationCommand

from threading import Lock

class LMControlNode(Node):
    def __init__(self):
        super().__init__('lm_control_node')
        self.lm_client = MD485DriverWrapper("/dev/ttyLM", 19200)
        self.lm_client.connect_lm()
        self.cmd_lock = Lock()

        #TODO change by robots 185
        self.offset_position = 103.9
        self.glob_speed = 5000

        self.elevation_initialize()

        self.input_grp = MutuallyExclusiveCallbackGroup()
        self.timer_grp = MutuallyExclusiveCallbackGroup()

        self.srv = self.create_service(ElevationCommand,'elevation/set_position',self.set_elevation_callback, callback_group=self.input_grp)

        self.subscriber = self.create_subscription(LMCommand,
                                                   'elevation/command',
                                                   self.topic_callback,
                                                   10,callback_group=self.input_grp)

        self.publisher = self.create_publisher(LMState,
                                               'elevation/state',
                                               10)
        
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback, callback_group=self.timer_grp)

        self.get_logger().info(f"LM control node init done.")

    def elevation_initialize(self):
        self.lm_client.reset_pose()
        self.lm_client.set_in_position_resolution()

        self.lm_client.set_velocity(-1800)

        while self.lm_client.read_io_status()[0]:
            time.sleep(0.1)

        time.sleep(0.5)
        self.lm_client.reset_pose()
        time.sleep(0.1)
        self.lm_client.set_position_with_velocity(56000,self.glob_speed)
        while True:
            ret=self.lm_client.read_moving_status()
            time.sleep(0.1)
            if ret:
                break



    def set_elevation_callback(self,request,response):
        self.get_logger().info(f"🔹 Received Elevation Service: Move {request.move} mm")

        move_step=int(self.lm_client.mm_to_count(request.move-self.offset_position))

        with self.cmd_lock:
            self.lm_client.set_position_with_velocity(move_step,self.glob_speed)
        
        if request.until_complete:
            while True:
                with self.cmd_lock:
                    ret=self.lm_client.read_moving_status()
                time.sleep(0.1)
                if ret:
                    break

        self.get_logger().info(f"🔹 Received Elevation Service Complete")
        response.successed=True
        return response


    def timer_callback(self):
        with self.cmd_lock:
            cur_pose = self.lm_client.count_to_mm(self.lm_client.read_monitor()[2]) + self.offset_position
            tar_pose = self.lm_client.count_to_mm(self.lm_client.read_target_position()) + self.offset_position
            #self.get_logger().info(f"Read normally")

        pub_msg = LMState()
        pub_msg.state = True
        pub_msg.current_position = cur_pose
        pub_msg.target_position = tar_pose
        self.publisher.publish(pub_msg)

    def topic_callback(self, msg):
        if msg.cmd_type == 'rel':
            with self.cmd_lock:
                self.lm_client.inc_position_with_velocity(int(self.lm_client.mm_to_count(msg.move)),self.glob_speed)
            




def main(args=None):
    rclpy.init(args=args)
    node = LMControlNode()

    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)

    try:
        node.get_logger().info("🚀 LMControlNode running with MultiThreadedExecutor")
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()