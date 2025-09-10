import rclpy
from rclpy.node import Node
import rclpy.task
import rclpy.wait_for_message
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import threading, time
import cv2
import os
import numpy as np
import json
from kcare_utils import *
from rclpy.executors import MultiThreadedExecutor

#from rclpy.callback_groups import ReentrantCallbackGroup

class ImageSaverNode(Node):
    def __init__(self):
        super().__init__('image_saver_node')
        
        # CV Bridge 초기화
        self.bridge = CvBridge()

        self.rbutils = RobotUtils(self)

        
        # 이미지 저장 경로를 상대 경로로 설정 (스크립트 위치 기준)
        script_dir = os.path.dirname(os.path.realpath(__file__))  # 현재 스크립트 위치
        self.output_dir = os.path.join(script_dir, '..', 'logs')  # logs 폴더 경로
        
        # logs 폴더가 없다면 생성
        os.makedirs(self.output_dir, exist_ok=True)
        
        # 파일 이름 카운터
        self.rgb_count = 0
        self.depth_count = 0
        self.robot_count =0 
        # 최근에 받은 이미지 저장
        self.rgb_image = None
        self.depth_image = None

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 이미지 구독자 추가
        self.rgb_sub = self.create_subscription(
            Image,
            '/femto/color/image_raw',
            self.rgb_callback,
            qos_profile
        )
        self.depth_sub = self.create_subscription(
            Image,
            '/femto/depth/image_raw',
            self.depth_callback,
            qos_profile
        )

    def rgb_callback(self, msg):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert RGB image: {e}")

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Failed to convert Depth image: {e}")


    def save_images(self):
        # 이미지 저장
        if self.rgb_image is not None:
            rgb_filename = os.path.join(self.output_dir, f'rgb_image_{self.rgb_count}.png')
            cv2.imwrite(rgb_filename, self.rgb_image)
            self.get_logger().info(f'Saved RGB image: {rgb_filename}')
            self.rgb_count += 1
        
        if self.depth_image is not None:
            depth_filename = os.path.join(self.output_dir, f'depth_image_{self.depth_count}.png')
            cv2.imwrite(depth_filename, self.depth_image)
            self.get_logger().info(f'Saved Depth image: {depth_filename}')
            self.depth_count += 1

        head_pose = self.rbutils.get_head_pose()

        if head_pose is not None:
            robot_json_filename = os.path.join(self.output_dir, f'robot_{self.robot_count}.json')
            head_state={
                    'rz':head_pose[0],
                    'ry':head_pose[1]}
            with open(robot_json_filename, 'w') as json_file:
                json.dump(head_state, json_file)
            self.get_logger().info(f'Saved roboy json: {robot_json_filename}')
            self.robot_count += 1


    def save_loop(self):
        #for ry in range(-45,15,5):
        #    self.rbutils.call_head_command([0.0,ry])
            #self.save_images()

        rz=-90.0
        for ry in range(-56,-39,1):
            self.rbutils.call_head_command([rz,ry])
            self.get_logger().info(f'Moved Head rz : {rz} ry : {ry}')
            pose=self.rbutils.get_head_pose()
            self.get_logger().info(f'Current Head Pose : {pose}')
            time.sleep(1)
            #image=self.get_latest_image()
            self.save_images()

        


def main(args=None):
    rclpy.init(args=args)
    master_node = ImageSaverNode()
    
    executor = MultiThreadedExecutor(num_threads=5)
    executor.add_node(master_node)
    executor.create_task(master_node.save_loop)
    try:
        master_node.get_logger().info("✅ Master Server is running...")
        executor.spin()


    except KeyboardInterrupt:
        pass
    finally:
        master_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()