# load_stcm_with_pose.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from slamware_ros_sdk.srv import SyncSetStcm

import os
from ament_index_python.packages import get_package_share_directory

class StcmMapLoaderWithPose(Node):
    def __init__(self):
        super().__init__('stcm_map_loader_with_pose')
        self.cli = self.create_client(SyncSetStcm, '/sync_set_stcm')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for sync_set_stcm service...')
        self.req = SyncSetStcm.Request()

        package_name = 'kcare_robot_ros2_controller'  # Replace with your actual package name!
        try:
            package_share_directory = get_package_share_directory(package_name)
            # Define the path to load the map from within the 'config' folder
            self.map_load_path = os.path.join(package_share_directory, 'config', 'slam_lab.stcm')
            self.get_logger().info(f"Attempting to load map from: {self.map_load_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to find package directory or set map load path: {e}")
            self.get_logger().error(f"Ensure your package '{package_name}' is correctly built and sourced.")
            self.map_load_path = None # Set to None if path resolution fails

    def send_request(self):
        if self.map_load_path is None:
            self.get_logger().error("Cannot load map: Map load path was not determined.")
            return

        # Load .stcm map file
        try:
            with open(self.map_load_path, "rb") as f:
                self.req.raw_stcm = f.read()
        except FileNotFoundError:
            self.get_logger().error(f"Map file not found at: {self.map_load_path}")
            return
        except Exception as e:
            self.get_logger().error(f"Failed to read map file from {self.map_load_path}: {e}")
            return

        # Set initial pose to (0.0, 0.0, yaw=0.0)
        self.req.robot_pose = Pose()
        self.req.robot_pose.position = Point(x=0.0, y=0.0, z=0.0)
        # Convert yaw to quaternion (yaw = 0.0 means facing x-direction)
        self.req.robot_pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=0.0,
            w=1.0
        )

        # Send request
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info("Map and initial pose successfully set")
        else:
            self.get_logger().error("Failed to load map with pose")

def main(args=None):
    rclpy.init(args=args)
    node = StcmMapLoaderWithPose()
    node.send_request()  # 불러올 맵 경로
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()