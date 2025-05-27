import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from slamware_ros_sdk.msg import GoHomeRequest, Line2DFlt32Array
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion
from visualization_msgs.msg import Marker
from kcare_robot_ros2_controller_msgs.srv import MobileMoveLabel
from kcare_robot_ros2_controller.src.pyutils.config_loader import load_robot_config, get_param
import math, os, json
from ament_index_python.packages import get_package_share_directory

class Mobile_Controller(Node):
    def __init__(self):
        super().__init__('mobile_control_node')

        # JSON config file loading
        robot_config = load_robot_config(
            package_name='kcare_robot_ros2_controller', # config 파일이 있는 패키지 이름
            config_file_env_var='ROBOT_NAME',
            default_robot_name='default',
            logger=self.get_logger() # 노드의 로거를 config_loader에 전달
        )

        self.poi_filename = get_param(robot_config, ['mobile', 'poi'], None, logger=self.get_logger())

        package_share_directory = get_package_share_directory('kcare_robot_ros2_controller')
        self.config_file_path = os.path.join(package_share_directory, 'config', self.poi_filename) # 파일명도 필요시 수정
        self.config = self.load_config(self.config_file_path)


    def load_config(self, config_path):
        """
        지정된 JSON 설정 파일을 로드합니다.
        """
        if not os.path.exists(config_path):
            self.get_logger().error(f"Config file not found at {config_path}")
            return None
        try:
            with open(config_path, 'r') as f:
                config_data = json.load(f)
            # self.get_logger().info(f"Successfully loaded config from {config_path}: {config_data}")
            return config_data
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse JSON config file {config_path}: {e}")
            return None
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred while loading config file {config_path}: {e}")
            return None


def main(args=None):
    rclpy.init(args=args)
    node = Mobile_Controller()

    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    # 노드내 함수 비동기 실행.초기화 함수등 실행
    try:
        node.get_logger().info("✅ Master Server is running...")
        executor.spin()  # ✅ 멀티스레드 실행
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()