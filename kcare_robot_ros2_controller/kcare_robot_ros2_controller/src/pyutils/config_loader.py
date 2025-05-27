import os
import json
from ament_index_python.packages import get_package_share_directory

# 로깅을 위한 더미 클래스 (Node를 상속받지 않는 모듈에서 로깅 사용 시)
# 실제 ROS2 노드에서는 rclpy.logging을 사용할 수 있습니다.
class DummyLogger:
    def info(self, msg):
        print(f"[INFO] {msg}")
    def warn(self, msg):
        print(f"[WARNING] {msg}")
    def error(self, msg):
        print(f"[ERROR] {msg}")

def load_robot_config(package_name: str, config_file_env_var: str = 'ROBOT_NAME', default_robot_name: str = 'default', logger=None) -> dict:
    """
    환경 변수를 기반으로 ROS2 패키지 내의 JSON 설정 파일을 로드합니다.

    Args:
        package_name (str): config 파일이 있는 ROS2 패키지의 이름.
        config_file_env_var (str): 환경 변수 이름 (예: 'ROBOT_NAME').
        default_robot_name (str): 환경 변수가 설정되지 않았을 때 사용할 기본 로봇 이름.
        logger: 로깅을 위한 객체 (예: rclpy.logging 또는 Node.get_logger()).

    Returns:
        dict: 로드된 JSON 설정 데이터. 오류 발생 시 빈 딕셔너리를 반환합니다.
    """
    if logger is None:
        logger = DummyLogger() # 로거가 제공되지 않으면 더미 로거 사용

    robot_name = os.getenv(config_file_env_var)
    if robot_name is None:
        logger.warn(f"환경 변수 '{config_file_env_var}'이 설정되지 않았습니다. 기본 로봇 이름 '{default_robot_name}'을 사용합니다.")
        robot_name = default_robot_name

    try:
        package_share_directory = get_package_share_directory(package_name)
        config_dir = os.path.join(package_share_directory, 'config')
        json_file_name = f"{robot_name}.json"
        json_config_path = os.path.join(config_dir, json_file_name)

        with open(json_config_path, 'r') as f:
            config_data = json.load(f)
            logger.info(f"'{json_file_name}' 설정 파일을 성공적으로 로드했습니다.")
            return config_data
    except FileNotFoundError:
        logger.error(f"설정 파일을 찾을 수 없습니다: {json_config_path}")
    except json.JSONDecodeError:
        logger.error(f"설정 파일 '{json_config_path}'의 JSON 형식이 올바르지 않습니다.")
    except Exception as e:
        logger.error(f"설정 파일 로딩 중 오류 발생: {e}")
    
    return {} # 오류 발생 시 빈 딕셔너리 반환

# 예시: 특정 파라미터를 안전하게 추출하는 헬퍼 함수
def get_param(config_data: dict, keys: list, default_value=None, logger=None):
    """
    딕셔너리에서 중첩된 키를 사용하여 값을 안전하게 가져옵니다.

    Args:
        config_data (dict): 설정 딕셔너리.
        keys (list): 키 목록 (예: ['elevation', 'offset']).
        default_value: 값이 없을 때 반환할 기본값.
        logger: 로깅 객체.

    Returns:
        any: 추출된 값 또는 기본값.
    """
    if logger is None:
        logger = DummyLogger()

    current_dict = config_data
    for i, key in enumerate(keys):
        if isinstance(current_dict, dict) and key in current_dict:
            current_dict = current_dict[key]
        else:
            logger.warn(f"경로 '{'.'.join(keys[:i+1])}'에 해당하는 키를 찾을 수 없습니다. 기본값 '{default_value}' 사용.")
            return default_value
    return current_dict