import rclpy
from rclpy.node import Node

import os
from ament_index_python.packages import get_package_share_directory

from slamware_ros_sdk.srv import SyncGetStcm


class StcmMapSaver(Node):
    def __init__(self):
        super().__init__('stcm_map_saver')
        self.cli = self.create_client(SyncGetStcm, '/sync_get_stcm')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for sync_get_stcm service...')
        self.req = SyncGetStcm.Request()
        package_name = 'kcare_robot_ros2_controller'
        try:
            package_share_directory = get_package_share_directory(package_name)
            # Define the path to save the map within the 'config' folder
            self.map_save_path = os.path.join(package_share_directory, 'config', 'slam_lab.stcm')
            self.get_logger().info(f"Map will be saved to: {self.map_save_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to find package directory or set map save path: {e}")
            self.get_logger().error(f"Ensure your package '{package_name}' is correctly built and sourced.")
            self.map_save_path = None # Set to None if path resolution fails

    def send_request(self):
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            with open("slam_lab.stcm", "wb") as f:
                f.write(future.result().raw_stcm)
            self.get_logger().info("Map saved to saved_map.stcm")
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