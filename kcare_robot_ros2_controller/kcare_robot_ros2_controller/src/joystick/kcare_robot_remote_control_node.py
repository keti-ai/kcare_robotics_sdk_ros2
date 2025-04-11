from rclpy.executors import MultiThreadedExecutor

from kcare_robot_ros2_controller.src.master.kcare_utils import *
from kcare_robot_ros2_controller.src.joystick.master_dxl_utils.dxl_wrapper import *

import time

class KcareRobotRemoteControlNode(Node):
    def __init__(self):
        super().__init__('kcare_master')
        self.rbutils=RobotUtils(self)
        self.driver = None
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()
        
        self.dxl_init()

        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.timer_callback,callback_group=self.timer_callback_group)

        self.chk_joint_home=False
        self.ready_arm=False

        


    def dxl_init(self):
        self.driver = RemoteDynamixelWrapper('/dev/ttyUSB0', 57600, [1, 2, 3, 4, 5, 6, 7, 8])
        self.driver.set_target_current(30)
        self.driver.set_torque_mode(True)
        target_pose = RobotParam.arm_ready
        target_pose.append(0.0)
        self.driver.set_target_joint(target_pose)



    def rb_init(self):
        # ROS Spin wait
        time.sleep(1)
        self.rbutils.call_motion_enable(8, 1)
        #로봇팔 포지션 제어모드
        self.rbutils.call_set_mode(0)
        #로봇 스테이트 셋
        self.rbutils.call_set_state(0)
        self.rbutils.call_set_servo_angle(RobotParam.arm_ready)
        self.rbutils.call_set_mode(6)
        self.rbutils.call_set_state(0)
        
        self.rbutils.xarm_set_tool_baudrate()
        self.rbutils.xarm_set_tool_timeout()
        self.rbutils.xarm_gripper_init()
        self.rbutils.xarm_set_motor_torque(50)

        self.rbutils.call_elevation_command(RobotParam.elev_home)
        self.get_logger().info('RB Init Done')
        self.ready_arm=True


    def check_joint_home(self):
        dxl_angle=self.driver.read_only_joints()[0]
        robot_angle=self.rbutils.get_robot_pose()['joint']
        #self.get_logger().info(f"Dynamixel : {dxl_angle}, Robot : {robot_angle}")
        max_joint_delta = 0.3
        abs_deltas = np.abs(robot_angle - dxl_angle)
        id_max_joint_delta = np.argmax(abs_deltas)
        if abs_deltas[id_max_joint_delta] < max_joint_delta:
            self.get_logger().info(f"Ready For teleop")
            self.chk_joint_home=True
        else:
            self.get_logger().info(f"ABS : {abs_deltas}")
        
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

    def remote_control(self):
        target_robot_angle, target_robot_gripper=self.driver.read_only_joints()

        target_robot_angle = list(self.clamp_joint_angles(target_robot_angle))
        target_robot_gripper = int(self.convert_angle_to_gripper(target_robot_gripper))
        self.get_logger().info(f"Target Angle : {target_robot_angle}, Target Gripper : {target_robot_gripper}")

        self.rbutils.call_set_servo_angle(target_robot_angle,speed=1.5,acc=10.0,wait=False)
        self.rbutils.xarm_set_finger_position(target_robot_gripper)
        
        #self.rbutils.call_gripper_command(target_robot_gripper,50)


    def timer_callback(self):
        if not self.ready_arm:
            self.rb_init()
            return
        if not self.chk_joint_home:
            self.check_joint_home()
        else:
            self.remote_control()

    def terminator(self):
        #self.rbutils.call_set_mode(0,wait=False)
        #time.sleep(0.1)
        #self.rbutils.call_set_mode(0,wait=False)
        self.rbutils.call_set_servo_angle(RobotParam.arm_home,wait=False)
        self.driver.set_torque_mode(False)
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
