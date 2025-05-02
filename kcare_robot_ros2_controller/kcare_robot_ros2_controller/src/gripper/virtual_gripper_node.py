import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

#For Robot tool Control
from kcare_robot_ros2_controller_msgs.msg import GripperState
from kcare_robot_ros2_controller_msgs.srv import GripperCommand

import time

class XarmToolGripper(Node):
    def __init__(self):
        super().__init__('virtual_gripper_node')
        
        self.srv_callback = MutuallyExclusiveCallbackGroup()
        self.gripper_service= self.create_service(GripperCommand,'gripper/command',self.set_gripperpose_callback,callback_group=self.srv_callback)
        
    def set_gripperpose_callback(self,request,response):
        self.get_logger().info(f"Gripper Service Call.")
        if (0 <= request.pose <= 1000 and 50<= request.force <= 100):
            #self.xarm_set_impedance_parameters(request.force)
            self.get_logger().info(f"Gripper Work Fine. Pose : {request.pose}, Force : {request.force}.")
            response.successed=True
        else:
            self.get_logger().info(f"Gripper request out of range.")
            response.successed=False
        return response
    
    
def main(args=None):
    rclpy.init(args=args)
    master_node = XarmToolGripper()
    
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(master_node)
    try:
        master_node.get_logger().info("✅ Virtual Gripper Server is running...")
        executor.spin()  # ✅ 멀티스레드 실행
    except KeyboardInterrupt:
        pass
    finally:
        master_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()