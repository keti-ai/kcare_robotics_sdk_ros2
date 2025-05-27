import rclpy
from rclpy.node import Node

import os
from ament_index_python.packages import get_package_share_directory # 폴백 경로를 위해 유지

from slamware_ros_sdk.srv import SyncGetStcm
from kcare_robot_ros2_controller.src.pyutils.config_loader import load_robot_config, get_param

class StcmMapSaver(Node):
    def __init__(self):
        super().__init__('stcm_map_saver')
        
        # JSON config file loading
        robot_config = load_robot_config(
            package_name='kcare_robot_ros2_controller', # config 파일이 있는 패키지 이름
            config_file_env_var='ROBOT_NAME',
            default_robot_name='default',
            logger=self.get_logger()
        )
        # JSON에서 맵 저장 파일 이름 가져오기
        saving_stcm_filename = get_param(robot_config, ['mobile', 'slam_map'], 'default_map.stcm', logger=self.get_logger())
        
        self.cli = self.create_client(SyncGetStcm, '/sync_get_stcm')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for sync_get_stcm service...')
        self.req = SyncGetStcm.Request()

        # 환경 변수가 설정되지 않았으면 설치된 패키지 공유 디렉토리에 저장 (폴백)
        try:
            package_share_directory = get_package_share_directory('kcare_robot_ros2_controller')
            self.map_save_path = os.path.join(package_share_directory, 'config', saving_stcm_filename)
        except Exception as e:
            self.get_logger().error(f"Failed to find package directory for fallback: {e}")
            self.map_save_path = None


    def send_request(self):
        if not self.map_save_path:
            self.get_logger().error("Map save path is not set. Aborting map saving.")
            return

        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            try:
                with open(self.map_save_path, "wb") as f:
                    f.write(future.result().raw_stcm)
                self.get_logger().info(f"Map saved to {self.map_save_path}")
            except IOError as e:
                self.get_logger().error(f"Failed to write map to {self.map_save_path}: {e}")
            except Exception as e:
                self.get_logger().error(f"An unexpected error occurred while saving map: {e}")
        else:
            self.get_logger().error("Failed to receive map data")

def main(args=None):
    rclpy.init(args=args)
    node = StcmMapSaver()
    node.send_request()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()