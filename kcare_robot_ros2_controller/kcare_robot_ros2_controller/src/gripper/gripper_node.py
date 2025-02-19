import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

from pymodbus.client import ModbusSerialClient
import time

from kcare_robot_ros2_controller_msgs.msg import GripperState
from kcare_robot_ros2_controller_msgs.srv import GripperCommand

class GripperControllerWrapper:
    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        
        # Initialize the pymodbus
        self.client = ModbusSerialClient(self.port, baudrate=self.baud)

    def connect_grip(self):
        self.client.connect()

    def disconnect_grip(self):
        self.client.close()

    def gripper_initialize(self):
        self.client.write_register(0,101,slave=1)
        time.sleep(0.1)
        self.client.write_register(0,213,slave=1)
        self.client.write_register(1,100,slave=1)
        time.sleep(0.1)

    def set_finger_position(self, position):
        self.client.write_register(0, 104, slave=1)
        self.client.write_register(1, position, slave=1)  # 목표 값 설정

    def set_motor_torque(self,ratio):
        self.client.write_register(0,212, slave=1)
        self.client.write_register(1,ratio,slave=1)


    def read_status(self):
        """
        모터 상태를 읽어 GripperState 메시지 형태로 반환하는 함수
        """
        try:
            # Modbus 요청 시 `slave=1`을 키워드 인자로 전달
            registers = self.client.read_holding_registers(0, count=5, slave=1)
            
            # 요청 실패 처리
            if not registers or registers.isError():
                print("⚠️ 모터 상태 읽기 실패!")
                return None
        except Exception as e:
            print(f"❌ 모터 상태 읽기 오류: {e}")
            return None

        # GripperState 메시지에 매핑
        gripper_state = GripperState()
        gripper_state.motor_position = registers.registers[1]
        gripper_state.motor_current = registers.registers[2]
        gripper_state.motor_velocity = registers.registers[3]
        gripper_state.finger_position = registers.registers[4]

        # 상태 플래그 (비트 마스킹 사용 가능)
        gripper_state.motor_enabled = bool(registers.registers[0] & 0x01)
        gripper_state.gripper_initialized = bool(registers.registers[0] & 0x02)
        gripper_state.position_ctrl_mode = bool(registers.registers[0] & 0x04)
        gripper_state.velocity_ctrl_mode = bool(registers.registers[0] & 0x08)
        gripper_state.current_ctrl_mode = bool(registers.registers[0] & 0x10)
        gripper_state.grp_opened = bool(registers.registers[0] & 0x20)
        gripper_state.grp_closed = bool(registers.registers[0] & 0x40)
        gripper_state.motor_fault = bool(registers.registers[0] & 0x80)

        return gripper_state



class GripperNode(Node):
    def __init__(self):
        super().__init__('gripper_subscriber')

        self.gripper_client = GripperControllerWrapper("/dev/ttyGripper", 115200)
        self.gripper_client.connect_grip()
        self.gripper_client.gripper_initialize()

        self.gripper_service= self.create_service(GripperCommand,'gripper/command',self.set_gripperpose_callback)

        self.publisher = self.create_publisher(GripperState,'gripper/state',10)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        

    def set_gripperpose_callback(self,request,response):
        self.get_logger().info(f"Gripper Service Call.")
        if (0 <= request.pose <= 1000 and 50<= request.force <= 100):
            self.gripper_client.set_motor_torque(request.force)
            self.gripper_client.set_finger_position(request.pose)
            self.get_logger().info(f"Gripper Work Fine. Pose : {request.pose}, Force : {request.force}.")
            response.successed=True
        else:
            self.get_logger().info(f"Gripper request out of range.")
            response.successed=False
        return response


    def timer_callback(self):
        # Gripper 상태 읽기
        gripper_state = self.gripper_client.read_status()
        if gripper_state:
            self.publisher.publish(gripper_state)
            #self.get_logger().info(f"Published Gripper State: {gripper_state}")


def main(args=None):
    rclpy.init(args=args)

    subscriber_ = GripperNode()
    rclpy.spin(subscriber_)

    subscriber_.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
