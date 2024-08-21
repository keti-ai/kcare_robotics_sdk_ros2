import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from slamware_ros_sdk.msg import GoHomeRequest, RobotBasicState
from kcare_robot_ros2_controller_msgs.msg import LMCommand, HeadCommand, HeadGoHomeRequest


class KcareRobotJoyControlNode(Node):

    def __init__(self):
        super().__init__('kcare_robot_control_node')

        # Subscribe to Joy topic
        self.subscriber_joy = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
        self.subscriber_joy  # prevent unused variable warning
        
        self.subscriber_slamware_robot_basic_state = self.create_subscription(
            RobotBasicState, 
            'slamware_ros_sdk_server_node/robot_basic_state',
            self.robot_basic_state_callback,
            10)
        self.subscriber_slamware_robot_basic_state

        # Publisher for topic
        self.publisher_base_move = self.create_publisher(Twist, 'cmd_vel', 10)
        self.publisher_go_home = self.create_publisher(GoHomeRequest, 'slamware_ros_sdk_server_node/go_home', 10)
        self.publisher_lm_move = self.create_publisher(LMCommand, 'elevation/command', 10)
        self.publisher_head_move = self.create_publisher(HeadCommand, 'head/command', 10)
        self.publisher_head_home = self.create_publisher(HeadGoHomeRequest, 'head/go_home', 10)
        
        
        # Timer for periodic execution
        timer_period = 0.1      # second
        self.timer_joy = self.create_timer(timer_period, self.timer_callback_joy)
        # self.timer_lm  = self.create_timer(timer_period, self.timer_callback_lm)
        
        # Variable to store the last received Joy message
        self.last_joy_msg = None

        self.joy_cmd_on = False
        
        # Default Twist message
        self.default_twist = Twist()
        self.default_twist.linear.x = 0.0
        self.default_twist.angular.z = 0.0
        
        self.get_logger().info(f"Kcare robot joy control node init done.")
        
    def joy_callback(self, msg):
        self.last_joy_msg = msg
        if msg.buttons[5]:
            self.joy_cmd_on = True
        else:
            self.joy_cmd_on = False
        
        if self.joy_cmd_on:
            if msg.buttons[3]:
                go_home_msg = GoHomeRequest()
                self.publisher_go_home.publish(go_home_msg)
                self.get_logger().info('Published GoHomeRequest')
            if msg.axes[-1] == 1:
                pass
            
            if msg.buttons[1]:
                head_home_msg = HeadGoHomeRequest()
                self.publisher_head_home.publish(head_home_msg)
                self.get_logger().info('Published HeadGoHomeRequest')
                
            # if msg.axes[-1] or msg.buttons[1]:
            #     lm_pose_msg = LMCommand()
            #     lm_pose_msg.cmd_type = 'rel'
            #     if msg.axes[-1] == 1:
            #         lm_pose_msg.move = 0.01    
            #     elif msg.axes[-1] == -1:
            #         lm_pose_msg.move = -0.01    
            #     else:
            #         lm_pose_msg.move = 0.0
                
            #     if msg.buttons[1]:
            #         lm_pose_msg.cmd_type = 'abs'
            #         lm_pose_msg.move = 0.76  
            #     self.publisher_lm_move.publish(lm_pose_msg)
                
    
    def robot_basic_state_callback(self, msg):
        battery_state = msg.battery_percentage
        # self.get_logger().info(f'Current battery: {battery_state} %')
    
    def timer_callback_joy(self):
        if self.last_joy_msg:
            if self.joy_cmd_on:
                self.default_twist.linear.x = self.last_joy_msg.axes[1]*0.3  # Adjust the axis index based on your joystick configuration
                self.default_twist.angular.z = self.last_joy_msg.axes[0]*0.5  # Adjust the axis index based on your joystick configuration
                
                self.publisher_base_move.publish(self.default_twist)
                # if self.default_twist.linear.x != 0.0 and self.default_twist.angular.z != 0.0:
                # self.get_logger().info(f'Published Twist from Joy: linear.x={self.default_twist.linear.x:.4f}, angular.z={self.default_twist.angular.z:.4f}')
                
                # if self.last_joy_msg.buttons[0]:
                #     self.lm_client.move_abs(0.6)
                
                
                # LM 가이드    
                lm_pose_msg = LMCommand()
                lm_pose_msg.cmd_type = 'rel'
                
                if self.last_joy_msg.axes[-1] == 1:
                    lm_pose_msg.move = 0.01    
                elif self.last_joy_msg.axes[-1] == -1:
                    lm_pose_msg.move = -0.01    
                else:
                    lm_pose_msg.move = 0.0
                self.publisher_lm_move.publish(lm_pose_msg)
                
                # 헤드
                head_pose_msg = HeadCommand()
                head_pose_msg.control_type = 'velocity'
                # head_pose_msg.control_type = 'position'
                
                head_pose_msg.rz  = self.last_joy_msg.axes[3] * 50
                head_pose_msg.ry = self.last_joy_msg.axes[4] * 50
                self.publisher_head_move.publish(head_pose_msg)

            else:
                self.default_twist.linear.x = 0.0
                self.default_twist.angular.z = 0.0
                # lm_pose_msg = LMCommand()
                # lm_pose_msg.position = 0.0
                # self.publisher_lm_move.publish(lm_pose_msg)
                # 헤드
                head_pose_msg = HeadCommand()
                head_pose_msg.control_type = 'velocity'
                # head_pose_msg.control_type = 'position'
                
                head_pose_msg.rz  = 0.0
                head_pose_msg.ry = 0.0
                self.publisher_head_move.publish(head_pose_msg)
        
    # def timer_callback_lm(self):
    #     if not self.lm_client.cur_abs_move:
    #         self.lm_client.move_rel(self.lm_move_rel)
        # lm_state = self.lm_client.read_motor_state()
        # self.get_logger().info(f"{lm_state}")
        # self.get_logger().info(f"Current LM Position: {self.lm_client.get_position()}")
    

def main(args=None):
    rclpy.init(args=args)
    
    node = KcareRobotJoyControlNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

