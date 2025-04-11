from rclpy.executors import MultiThreadedExecutor

from kcare_robot_ros2_controller.src.master.kcare_utils import *

import time


class KcareMaster(Node):
    def __init__(self):
        super().__init__('kcare_master')

        self.rbutils=RobotUtils(self)
        self.srv_callback_group = MutuallyExclusiveCallbackGroup()

    def rb_init(self):
        # ROS Spin wait
        time.sleep(1)
        #self.rbutils.call_gripper_command(RobotParam.grip_open)
        self.rbutils.call_head_command([20.0, 0.0])
        #로봇 전원 On
        self.rbutils.call_motion_enable(8, 1)
        #로봇팔 포지션 제어모드
        self.rbutils.call_set_mode(0)
        #로봇 스테이트 셋
        self.rbutils.call_set_state(0)
        #조인트 기반 홈자세 이동
        
        #self.rbutils.xarm_set_tool_baudrate()
        #self.rbutils.xarm_set_tool_timeout()
        #self.rbutils.xarm_gripper_init()

        self.rbutils.xarm_set_motor_torque(50)
        
        self.rbutils.xarm_set_finger_position(1000)

        time.sleep(2)
        
        self.rbutils.xarm_set_finger_position(500)

        self.rbutils.call_set_servo_angle(RobotParam.arm_home)
        self.rbutils.call_elevation_command(RobotParam.elev_home)
        self.rbutils.call_head_command([-20.0, 0.0])
        
def main(args=None):
    rclpy.init(args=args)
    master_node = KcareMaster()
    
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
        #rclpy.shutdown()

if __name__ == '__main__':
    main()


