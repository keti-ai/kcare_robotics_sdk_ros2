import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState
from kcare_robot_ros2_controller_msgs.msg import LMCommand, LMState
from kcare_robot_ros2_controller_msgs.srv import ElevationCommand

from threading import Lock
from kcare_robot_ros2_controller.src.elevation.md_utils.MD485driver import MD485DriverWrapper

import time

class LMControlNode(Node):
    def __init__(self):
        super().__init__('lm_control_node')


        self.declare_parameter('port','/dev/ttyUSB0')
        self.declare_parameter('baudrate',19200)
        self.declare_parameter('offset_position',0.0)
        self.declare_parameter('global_speed',100.0)
        self.declare_parameter('initial_speed',10.0)
        self.declare_parameter('elevation_range',[0.0, 700.0])
        self.declare_parameter('home_position',10.0)
        self.declare_parameter('encoder_ppr', 16384)
        self.declare_parameter('reduction_ratio', 28/19)
        self.declare_parameter('lead_pitch', 10)
        self.declare_parameter('rpm_range', [-3000, 3000])
        self.declare_parameter('limit_inveted', False)
        self.declare_parameter('motor_inv',True)



        #self.get_logger().info(f'Home position: {self.get_parameter("home_position").get_parameter_value().double_value}')

        self.lm_client = MD485DriverWrapper(
            port=self.get_parameter('port').get_parameter_value().string_value,
            baud=self.get_parameter('baudrate').get_parameter_value().integer_value,
            ros_node=self,
            debug=True,
            elevation_range=self.get_parameter('elevation_range').get_parameter_value().double_array_value,
            offset_position=self.get_parameter('offset_position').get_parameter_value().double_value,
            encoder_ppr=self.get_parameter('encoder_ppr').get_parameter_value().integer_value,
            reduction_ratio=self.get_parameter('reduction_ratio').get_parameter_value().double_value,
            lead_pitch=self.get_parameter('lead_pitch').get_parameter_value().integer_value,
            rpm_range=self.get_parameter('rpm_range').get_parameter_value().integer_array_value,
            limit_inveted=self.get_parameter('limit_inveted').get_parameter_value().bool_value,
            motor_inv=self.get_parameter('motor_inv').get_parameter_value().bool_value,
        )
        self.lm_client.connect_lm()

        self.cmd_callback_group = MutuallyExclusiveCallbackGroup()
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()
        self.cmd_lock = Lock() # Prevent command overlap
        
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback, callback_group=self.timer_callback_group)
        self.state_publisher = self.create_publisher(LMState,
                                        'elevation/state',
                                        10)
        self.joint_publisher = self.create_publisher(JointState,
                                        'elevation/joint_states',
                                        10)
        
        self.srv_set_position = self.create_service(ElevationCommand,'elevation/set_position',self.set_elevation_callback, callback_group=self.cmd_callback_group)
        
        self.subscriber = self.create_subscription(LMCommand,
                                                   'elevation/command',
                                                   self.topic_callback,
                                                   10,callback_group=self.cmd_callback_group)
        
        
        self.initialized=False
        self.current_position = 0.0
        self.target_position = 0.0
        self.emergency_stop = False
        
        self.initialize_elevation()
        self.get_logger().info(f"Elevation Initialize Complete")
        
    def initialize_elevation(self):
        if self.lm_client.read_init_set_ok():
            self.get_logger().info("Elevation already initialized.")
            self.initialized = True
            return
        self.get_logger().info("Initializing elevation...")
        
        self.lm_client.reset_pose()
        self.lm_client.set_in_position_resolution(0.5)
        self.lm_client.set_linear_velocity(-10.0)
        
        while self.lm_client.get_lower_limit_switch():
            time.sleep(0.1)
            
        self.lm_client.reset_pose()
        time.sleep(0.05)
        
        self.lm_client.move_abs_pose_mm(
            self.get_parameter('home_position').get_parameter_value().double_value,
            self.get_parameter('global_speed').get_parameter_value().double_value
        )

        while not self.lm_client.read_in_position_status():
            time.sleep(0.1)
        
        self.lm_client.set_init_set(True)
        self.initialized=True


        
    def read_loop(self):
        with self.cmd_lock:
            self.lm_client.read_current_position()
            self.lm_client.read_target_position()
            self.lm_client.get_emergency_stop()
            
        if not self.lm_client.get_emergency_stop():
            self.get_logger().error("Emergency Stop Activated! Stopping all movements.")
        else:
            self.get_logger().debug(f"Current Position: {self.current_position:.2f} mm, Target Position: {self.target_position:.2f} mm")     
            
    def data_publish(self):       
        state_msg = LMState()
        state_msg.current_position = self.current_position
        state_msg.target_position = self.target_position
        state_msg.state = (self.emergency_stop == 1)
        
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.header.frame_id = 'elevation_base'
        joint_msg.name = ['elevation_joint']
        joint_msg.position = [self.current_position / 1000.0]  # Convert mm to meters
        
        self.state_publisher.publish(state_msg)
        self.joint_publisher.publish(joint_msg)

    def set_elevation_callback(self,request,response):
        self.get_logger().info(f"🔹 Received Elevation Service: Move {request.move} mm")

        target_absolute_position = request.move # request.move가 mm 단위의 절대 위치라고 가정
        elev_range = self.get_parameter('elevation_range').value

        if not (elev_range[0] <= target_absolute_position <= elev_range[1]):
            self.get_logger().warn(
                f"Requested move position ({request.move} mm) is outside the valid range "
                f"{elev_range} mm. Aborting move."
            )
            response.successed = False # 성공하지 않았음을 명시
            return response
        

        # 2. 유효한 범위 내에 있다면 이동 로직 실행 (기존 로직)
        self.lm_client.move_abs_pose_mm(target_absolute_position,self.get_parameter('global_speed').get_parameter_value().double_value)


        if request.until_complete:
            while True:
                with self.cmd_lock:
                    ret=self.lm_client.read_in_position_status()
                time.sleep(0.1)
                if ret:
                    self.get_logger().info("Target Move Complete.")
                    break
                if self.emergency_stop==0:
                    self.get_logger().error("Emergency Stop Activated during movement! Aborting.")
                    response.successed = False
                    return response
                
        response.successed=True
        return response
        
    def topic_callback(self, msg):
        if self.emergency_stop==0:
            return
        if msg.cmd_type == 'rel':
            with self.cmd_lock:
                self.lm_client.move_inc_pose_mm(msg.move,self.get_parameter('global_speed').get_parameter_value().double_value)

    def timer_callback(self):
        if not self.initialized:
            return
        self.read_loop()
        self.data_publish()

def main(args=None):
    rclpy.init(args=args)
    node = LMControlNode()

    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)

    try:
        node.get_logger().info("LMControlNode running with MultiThreadedExecutor")
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()