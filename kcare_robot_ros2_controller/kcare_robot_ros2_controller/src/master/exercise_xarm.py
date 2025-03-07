from rclpy.executors import MultiThreadedExecutor
from kcare_robot_ros2_controller.src.master.kcare_utils import *

class ExerciseNode(Node):
    def __init__(self):
        super().__init__('exercise')

        self.rbutils=RobotUtils(self)

    def init_motion(self):
        # 로봇 전원 On
        self.rbutils.call_motion_enable(8, 1)
        # 로봇팔 포지션 제어모드
        self.rbutils.call_set_mode(0)
        # 로봇 스테이트 셋
        self.rbutils.call_set_state(0)
        # 조인트 기반 홈자세 이동
        self.rbutils.call_set_servo_angle(RobotParam.arm_home)
        # 조인트 기반 픽업자세 이동
        self.rbutils.call_set_servo_angle(RobotParam.arm_ready)
        # 베이스좌표계 기반 상대 움직임
        self.rbutils.call_set_relative_robot_pose(dx=30.0)
        self.rbutils.call_set_relative_robot_pose(dy=30.0)


def main(args=None):
    rclpy.init(args=args)
    # 노드 선언
    node = ExerciseNode()
    # 토픽 콜백 데이터를 멈추지 않고 실시간으로 받기 위해 실행자 스레드 정의
    executor = MultiThreadedExecutor(num_threads=3)
    # 노드 시작
    executor.add_node(node)
    # 노드내 함수 비동기 실행.초기화 함수등 실행
    executor.create_task(node.init_motion)
    try:
        node.get_logger().info("Node is running...")
        executor.spin()  
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()