import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from pymodbus.client import ModbusSerialClient

import concurrent.futures

from kcare_robot_ros2_controller_msgs.msg import LMCommand, LMState


class LMControllerWrapper:
    def __init__(self, port, baud):
        self.port = port
        self.baud = baud

        self.client = ModbusSerialClient(self.port, baudrate=self.baud)

        self.stopped = True

        self.limit_position = (0.0, 0.08)  # 제어 범위 설정 meter(unit)
        self.current_position = 0.08  # unit: Meter
        self.target_position = 0.08  # unit: Meter

        self.limit_step = (0, 2918400)
        self.current_step = 0  # unit: Step
        self.target_step = 0  # unit: Step
        self.velocity_step = 2048000

        self.state = False

    def connect_lm(self):
        # 포트 열기
        self.client.connect()

    def disconnect_lm(self):
        # 포트 닫기
        self.client.close()

    def set_brake(self, bBrake):
        # 전자 브레이크 활성황
        if bBrake:
            self.client.write_register(0x002F, 0x00, slave=1)
            self.stopped = True
            # print(f"LM Brake Enabled.")
        else:
            self.client.write_register(0x002F, 0x0A, slave=1)
            self.stopped = False
            # print(f"LM Brake Disabled.")

    def motor_stop(self, bEna=True):
        if bEna:
            self.client.write_register(0x002D, 0x0000, slave=1)
        else:
            self.client.write_register(0x002D, 0x0001, slave=1)

    def read_motor_state(self):
        # 모터 동작 상태 체크. 읽어들인 byte의 15번째 주소 값이 0이면 모터 정지. 1이면 모터 작동
        buf = self.client.read_holding_registers(0x002E, count=1, slave=1)
        if buf.isError():
            return None
        value = buf.registers[0]
        bit_value = (value >> 15) & 1
        self.state = not bit_value
        return self.state

    def reset_speed(self, speed):
        # 원점복귀 속도 제어 한바퀴 51200
        if speed > 512000:
            speed = 512000
        elif speed < 25600:
            speed = 25600
        else:
            speed = speed
        high, low = self.split_32bit_to_16bit(speed)
        self.client.write_registers(0x0024, [high, low], slave=1)

    def run_speed(self, speed):
        # 구동모드 속도 제어. 한바퀴 51200
        if speed > 512000:
            speed = 512000
        elif speed < 25600:
            speed = 25600
        else:
            speed = speed
        high, low = self.split_32bit_to_16bit(speed)
        self.client.write_registers(0x0028, [high, low], slave=1)

    def move(self, position):
        high_byte = (position >> 16) & 0xFFFF
        low_byte = position & 0xFFFF

        high_fixed = 0x0F00 | high_byte
        low_fixed = low_byte
        self.client.write_registers(0x002B, [high_fixed, low_fixed], slave=1)

    def read_current_step(self):  ##현재 스탭각 취득
        ret = self.client.read_holding_registers(0x002B, count=2, slave=1)

        def get_steps(high_byte, low_byte):
            high_byte = high_byte & 0x7FFF
            combined = (high_byte << 16) | low_byte
            return combined

        return get_steps(ret.registers[0], ret.registers[1])

    def initializing(self):
        # 포토센서기반 리니어 가이드 위치 초기화
        self.set_brake(False)  # 브레이크 해제
        self.reset_speed(204800)  # 초기위치모드 구동속도 조절
        self.client.write_register(0x002A, 0x100F, slave=1)  # 속도모드 아래 회전 원점찾기

        # 모터 정지 상태 확인
        time.sleep(0.5)
        while self.read_motor_state():
            time.sleep(0.05)

        self.set_brake(True)  # 브레이크 동작

        self.run_speed(self.velocity_step)

    def split_32bit_to_16bit(self, value):
        # 바이값을 2개의 배열로 변경
        high = (value >> 16) & 0xFFFF
        low = value & 0xFFFF
        return high, low

    def step_to_meter(self, step):
        meter = step * 0.01 / 51200
        return meter

    def meter_to_step(self, meter):
        step = meter * (51200 / 0.01)
        return step


class LMControlNode(Node):
    def __init__(self):
        super().__init__('lm_control_node')
        self.lm_speed = 0.06  # m/s
        self.lm_move_rel = 0.0  # meter

        self.lm_client = LMControllerWrapper("/dev/ttyACM0", 9600)
        self.lm_client.connect_lm()
        self.lm_client.initializing()

        self.subscriber = self.create_subscription(LMCommand,
                                                   'elevation/command',
                                                   self.topic_callback,
                                                   10)

        self.publisher = self.create_publisher(LMState,
                                               'elevation/state',
                                               10)

        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(f"LM control node init done.")

    def timer_callback(self):
        if self.lm_client.target_step == self.lm_client.current_step:
            # self.lm_client.set_brake(True)
            pass
        else:
            self.lm_client.set_brake(False)
            #self.lm_client.run_speed(self.lm_client.velocity_step)
        self.lm_client.move(self.lm_client.target_step)
        self.lm_client.current_step = self.lm_client.read_current_step()

        #스텝각을 미터로 변환
        self.lm_client.target_position=self.lm_client.step_to_meter(self.lm_client.target_step)
        self.lm_client.current_position = self.lm_client.step_to_meter(self.lm_client.current_step)

        pub_msg = LMState()
        pub_msg.state = self.lm_client.state
        pub_msg.current_position = self.lm_client.current_position
        pub_msg.target_position = self.lm_client.target_position
        self.publisher.publish(pub_msg)

    def topic_callback(self, msg):
        self.get_logger().info(f"LM move_type: {msg.cmd_type}, move: {msg.move:3f} mm")
        move_step=int(self.lm_client.meter_to_step(msg.move))
        print(move_step)
        if msg.cmd_type == 'rel':
            if self.lm_client.stopped:
                self.lm_client.set_brake(False)
            self.lm_client.target_step=self.lm_client.current_step+move_step
        elif msg.cmd_type == 'abs':
            if self.lm_client.stopped:
                self.lm_client.set_brake(False)
            self.lm_client.target_step=move_step


def main(args=None):
    rclpy.init(args=args)
    node = LMControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()