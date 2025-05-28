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

        self.topic_sub_group = MutuallyExclusiveCallbackGroup()
        self.srv_callback_group = MutuallyExclusiveCallbackGroup()

        TOPIC_SUBS = {
            'robot_pose': ('/robot_pose', PoseStamped,self.robot_pose_callback),
            'virtual_wall':('/virtual_walls',Line2DFlt32Array,self.virtual_wall_callback)
        }

        TOPIC_PUBS = {
            'cmd_vel':('/cmd_vel',Twist),
            'goal_pose':('/move_base_simple/goal',PoseStamped),
            'go_home':('/slamware_ros_sdk_server_node/go_home',GoHomeRequest),
            'virtual_marker':('/virtual_marker',Marker),
        }
        
        self.topic_subs = {}
        for topic_tag, (topic_name, topic_type,topic_callback_fun) in TOPIC_SUBS.items():
            self.topic_subs[topic_tag] = self.create_subscription(topic_type,topic_name,topic_callback_fun,10,callback_group=self.topic_sub_group)
            self.get_logger().info(f"Subscriber created: {topic_tag} -> {topic_name} with {topic_callback_fun}")
        
        # 퍼블리셔 등록
        self.topic_pubs = {}
        for topic_tag, (topic_name, topic_type) in TOPIC_PUBS.items():
            self.topic_pubs[topic_tag] = self.create_publisher(topic_type, topic_name, 10)
            self.get_logger().info(f"Publisher created: {topic_tag} -> {topic_name}")

        self.service_client = self.create_service(MobileMoveLabel,'mobile/goal_pose',self.service_callback_pose, callback_group=self.srv_callback_group)

    def publish_goal_pose(self, point_x, point_y, target_yaw):
        goal = PoseStamped()
        goal.header.frame_id = ""  # 또는 사용 중인 좌표계 ("odom" 등)
        goal.header.stamp = self.get_clock().now().to_msg()

        # 위치 설정
        goal.pose.position.x = point_x
        goal.pose.position.y = point_y
        goal.pose.position.z = 0.0

        # 방향 설정 (target_yaw 값을 사용)
        # 쿼터니언 계산
        goal.pose.orientation.z = math.sin(target_yaw / 2.0)
        goal.pose.orientation.w = math.cos(target_yaw / 2.0)
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0

        # 퍼블리시
        self.topic_pubs['goal_pose'].publish(goal)
        self.get_logger().info(
            f"Goal published to (x={goal.pose.position.x:.2f}, y={goal.pose.position.y:.2f}, yaw={target_yaw:.2f} rad)"
        )

    def robot_homing(self):
        msg=GoHomeRequest()
        self.topic_pubs['go_home'].publish(msg)

    def robot_pose_callback(self, msg: PoseStamped):
        # 위치
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        # 쿼터니언
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

    def virtual_wall_callback(self, msg):
        for line in msg.lines:
            line_id = line.id  # 각 선의 ID
            start = line.start
            end = line.end

        marker = Marker()
        marker.header.frame_id = 'slamware_map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = 0
        marker.type = Marker.LINE_LIST  # 여러 선분 표현!
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # 선 굵기

        # 벽 색깔 설정 (예: 빨간색)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.points = []
        for line in msg.lines:
            start = line.start  # geometry_msgs/Point32
            end = line.end      # geometry_msgs/Point32

            p_start = Point(x=start.x, y=start.y, z=0.0)
            p_end = Point(x=end.x, y=end.y, z=0.0)

            marker.points.append(p_start)
            marker.points.append(p_end)


        self.topic_pubs["virtual_marker"].publish(marker)

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

    def service_callback_pose(self, request: MobileMoveLabel.Request, response: MobileMoveLabel.Response):
        if self.config is None or "locations" not in self.config:
            self.get_logger().error("Configuration not loaded or 'locations' key missing. Cannot process request.")
            response.successed = False
            return response
        
        target_label = request.label.lower() # 요청된 레이블을 소문자로 변환
        self.get_logger().info(f"Received move request for label: '{target_label}'")

        locations = self.config.get("locations", {}) # locations 키가 없을 경우 빈 dict 반환
        
        if target_label in locations:
            action_config = locations[target_label]
            action_type = action_config.get("type")

            if action_type == "homing":
                self.robot_homing()
            elif action_type == "pose":
                x = action_config.get("x")
                y = action_config.get("y")
                theta = action_config.get("theta")
                if x is not None and y is not None and theta is not None:
                    self.publish_goal_pose(x, y, theta)
                    response.successed = True
                else:
                    self.get_logger().error(f"Pose parameters (x, y, theta) missing or invalid for label: '{target_label}' in config.")
                    response.successed = False
            else:
                self.get_logger().warn(f"Unknown action type '{action_type}' for label: '{target_label}' in config.")
                response.successed = False
        else:
            self.get_logger().warn(f"Label '{target_label}' not found in configuration.")
            response.successed = False
        return response


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