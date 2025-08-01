from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState
from kcare_robot_ros2_controller.src.master.kcare_utils import *
from kcare_robot_ros2_controller.src.joystick.master_dxl_utils.dxl_wrapper import *

import time

JOINT_LIMITS_DEG = {
    0: (90-50, 90+35),  # Joint 0: min, max
    1: (15-20, 15+110),  # Joint 1: min, max
    2: (0-90, 0+90),  # Joint 2: min, max
    3: (0, 0+135),  # Joint 3: min, max
    4: (-180-100, 180+100),  # Joint 4: min, max
    5: (0-92, 0+95),  # Joint 5: min, max
    6: (-90-135, -90+135),  # Joint 6: min, max
    7: (-30, 0) # Gripper min, max
}

JOINT_LIMITS = {i: np.deg2rad(limits) for i, limits in JOINT_LIMITS_DEG.items()}


class KcareRobotRemoteControlNode(Node):
    def __init__(self):
        super().__init__('kcare_master')
        self.rbutils=RobotUtils(self)
        
        self.topic_callback_grp=MutuallyExclusiveCallbackGroup()

        self.master_joint_subscriber=self.create_subscription(
            JointState,
            '/master/joint_states',
            self.master_callback,
            1,callback_group=self.topic_callback_grp)

        self.chk_joint_home=False
        self.ready_arm=False
        

    def rb_init(self):
        # ROS Spin wait
        self.rbutils.call_motion_enable(8, 1)
        #로봇팔 포지션 제어모드
        self.rbutils.call_set_mode(0)
        #로봇 스테이트 셋
        self.rbutils.call_set_state(0)
        
        self.rbutils.call_set_servo_angle(RobotParam.arm_ready)
        self.rbutils.call_set_mode(6)
        self.rbutils.call_set_state(0)
        
        self.get_logger().info('RB Init Done')
        self.ready_arm=True

    def check_joint_home(self,dxl_angles):
        robot_angle=RobotParam.arm_ready
        max_joint_delta = 0.3
        abs_deltas = np.abs(robot_angle - dxl_angles)
        id_max_joint_delta = np.argmax(abs_deltas)
        if abs_deltas[id_max_joint_delta] < max_joint_delta:
            self.get_logger().info(f"Ready For teleop")
            self.chk_joint_home=True
        else:
            self.get_logger().info(f"ABS : {abs_deltas}")
            
    def remote_control(self,robot_angle,gripper_pose):
        
        self.get_logger().info(f"Target Angle : {robot_angle}, Target Gripper : {gripper_pose}")

        self.rbutils.call_set_servo_angle(list(robot_angle),speed=1.5,acc=10.0,wait=False)

        self.rbutils.call_gripper_command(int(gripper_pose),50)


    def clamp_joint_angles(self, angles):
        """각도를 허용된 범위로 클램핑"""
        # 입력 각도가 리스트일 경우 numpy 배열로 변환
        if isinstance(angles, list):
            angles = np.array(angles) 
        clamped_angles = np.copy(angles)
        for i in range(len(angles)):
            min_angle, max_angle = JOINT_LIMITS[i]
            if angles[i] < min_angle:
                clamped_angles[i] = min_angle
            elif angles[i] > max_angle:
                clamped_angles[i] = max_angle
        return clamped_angles

    def convert_angle_to_gripper(self,angle):
        # Set Gripper Pose
        dxl_min = -25 * np.pi / 180
        dxl_max = 0 * np.pi / 180
        new_min = 0
        new_max = 1000

        if angle > dxl_max:
            angle = dxl_max
        elif angle < dxl_min:
            angle = dxl_min
        mapped_value = (angle - dxl_min) / (dxl_max - dxl_min) * (new_max - new_min) + new_min
        
        return mapped_value


    def master_callback(self,msg):
        
        if not self.ready_arm:
            self.rb_init()
            return
        
        self.get_logger().info(f"--- Received JointState ---")
        self.get_logger().info(f"Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")
        
        target_robot_angle=self.clamp_joint_angles(msg.position[0:7])
        target_robot_gripper=self.convert_angle_to_gripper(msg.position[7])
        
        
        self.get_logger().info(f"Position: {target_robot_angle}")
        self.get_logger().info(f"Gripper: {target_robot_gripper}")
        
        if not self.chk_joint_home:
            self.check_joint_home(target_robot_angle)
        else:
            self.remote_control(target_robot_angle,target_robot_gripper)

    def terminator(self):
        #self.rbutils.call_set_mode(0,wait=False)
        #time.sleep(0.1)
        #self.rbutils.call_set_mode(0,wait=False)
        self.rbutils.call_set_servo_angle(RobotParam.arm_ready,wait=False)
        #self.driver.close()

def main(args=None):
    rclpy.init(args=args)
    node = KcareRobotRemoteControlNode()
    
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    # 노드내 함수 비동기 실행.초기화 함수등 실행
    #executor.create_task(node.rb_init)
    try:
        node.get_logger().info("✅ Master Server is running...")
        executor.spin()  # ✅ 멀티스레드 실행
    except KeyboardInterrupt:
        node.terminator()
    finally:
        #node.terminator()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
