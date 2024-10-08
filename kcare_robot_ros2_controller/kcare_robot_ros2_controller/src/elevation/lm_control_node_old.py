import time

import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from pymodbus.client import ModbusSerialClient

import concurrent.futures

from kcare_robot_ros2_controller_msgs.msg import LMCommand, LMState


class LMControllerWrapper:
    def __init__(self,port,baud):
        self.port = port
        self.baud = baud

        self.client = ModbusSerialClient(self.port, baudrate=self.baud)

        self.stopped = False

        self.limit_position = (0.0, 0.08)
        self.current_position = 0.08     # unit: Meter
        self.target_position = 0.08      # unit: Meter

        self.limit_step = (0, 2918400)
        self.current_step = 0           # unit: Step
        self.target_step = 0            # unit: Step

        self.cur_abs_move = False
        self.speed = 0

        self.state = False

    def connect_lm(self):
        self.client.connect()

    def disconnect_lm(self):
        self.client.close()

    def initializing(self):
        self.set_brake(False)

        # ret = self.client.write_register(0x002F,0x0F,slave=0)

        self.client.write_registers(0x0024, [0x0003, 0x2000], slave=0)  # 빠른거 구동속도
        # self.client.write_registers(0x0024, [0x0000, 0xC800], slave=0)  # 느린거 구동속도

        # self.client.write_register(0x002A, 0x110F, slave=0)  #위 역회전 원점찾기
        self.client.write_register(0x002A, 0x101F, slave=0)  #아래 회전 원점찾기

    def stop(self):
        pass

    def split_32bit_to_16bit(self,value):
        high = (value>>16) & 0xFFFF
        low = value & 0xFFFF
        return high, low

    def set_vel(self, vel):
        step_speed = int(vel * 1000 * 5120)

        if step_speed > 512000:
            self.speed = 512000
        elif step_speed < 25600:
            self.speed = 25600
        else:
            self.speed = step_speed

    def set_brake(self,bBrake):
        if bBrake:
            self.client.write_register(0x002F,0x00,slave=0)
            # print(f"LM Brake Enabled.")
        else:
            self.client.write_register(0x002F,0x0A,slave=0)
            # print(f"LM Brake Disabled.")

    def get_vel(self):
        return self.speed / 5120 / 1000

    def move(self, position):
        # self.set_brake(False)

        high, low = self.split_32bit_to_16bit(self.speed)

        ret = self.client.write_registers(0x0028, [high, low], slave=0)  # 구동속도

        high_byte = (position >> 16) & 0xFFFF
        low_byte = position & 0xFFFF

        # high_fixed = 0x0F00 | high_byte
        high_fixed = 0x8F00 | high_byte
        low_fixed = low_byte

        ret = self.client.write_registers(0x002B, [high_fixed, low_fixed], slave=0)

        # self.set_brake(True)

        # current_step = self.client.read_holding_registers(0x002B, 2)
        # print(current_step.registers)
        # self.wait_done()

    def move_abs(self, position):

        self.cur_abs_move = True

        if position > self.limit_position[0]:
            self.target_position = self.limit_position[0]
        elif position < self.limit_position[1]:
            self.target_position = self.limit_position[1]
        else:
            self.target_position = position

        # self.target_step = (self.limit_position[1] - self.target_position) * 1000 * 5120
        self.target_step = (self.target_position - self.limit_position[1]) * 1000 * 5120
        self.target_step = int(self.target_step)

        if(self.read_current_step() == self.target_step):
            self.set_brake(True)
        else:
            self.set_brake(False)

        self.move(self.target_step)

        self.current_position = self.target_position
        self.current_step = self.target_step

        self.cur_abs_move = False


    def move_rel(self, position):
        self.target_position += position

        if self.target_position > self.limit_position[0]:
            self.target_position = self.limit_position[0]
        elif self.target_position < self.limit_position[1]:
            self.target_position = self.limit_position[1]

        # self.target_step = (self.limit_position[1] - self.target_position) * 1000 * 5120
        self.target_step = (self.target_position - self.limit_position[1]) * 1000 * 5120
        self.target_step = int(self.target_step)
        # print(self.read_current_step())
        # print(self.target_step)

        #if(self.read_current_step() == self.target_step):
        #    self.set_brake(True)
        #else:
        #    self.set_brake(False)
        self.move(self.target_step)

        self.current_position = self.target_position
        self.current_step = self.target_step

    def get_position(self):
        return self.limit_position[0] - (self.current_step / 5120 / 1000)

    def read_current_step(self):    ##현재 스탭각 취득
        ret=self.client.read_holding_registers(0x002B,count=2,slave=0)

        def get_steps(high_byte,low_byte):
            high_byte=high_byte & 0x7FFF
            combined=(high_byte<<16)|low_byte
            return combined

        return get_steps(ret.registers[0],ret.registers[1])

    def read_motor_state(self):
        buf = self.client.read_holding_registers(0x002E, count=1, slave=0)

        if buf.isError():
            return None

        value = buf.registers[0]

        bit_value = (value >> 15) & 1

        self.state = not bit_value
        return self.state

    def wait_done(self):
        time.sleep(0.5)
        while self.read_motor_state():
            time.sleep(0.05)


class LMControlNode(Node):
    def __init__(self):
        super().__init__('lm_control_node')
        self.lm_speed = 0.03        # m/s
        self.lm_move_rel = 0.0      # meter

        self.lm_client = LMControllerWrapper("/dev/ttyLM", 9600)
        self.lm_client.connect_lm()
        self.lm_client.initializing()
        self.lm_client.wait_done()
        #self.lm_client.set_brake(True)
        self.lm_client.set_vel(self.lm_speed)

        self.subscriber = self.create_subscription(LMCommand,
                                                   'elevation/command',
                                                   self.topic_callback,
                                                   10)

        self.publisher = self.create_publisher(LMState,
                                               'elevation/state',
                                               10)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(f"LM control node init done.")

    def timer_callback(self):
        #Current step과 Target Step이 같으면 Brake Enable
        # if(self.lm_client.read_current_step() == self.lm_client.target_step):
        #     self.lm_client.set_brake(True)

        # TODO            current position
        pub_msg = LMState()
        pub_msg.state = self.lm_client.state
        pub_msg.current_position = 0.0
        pub_msg.target_position = self.lm_client.target_position
        self.publisher.publish(pub_msg)


    def topic_callback(self, msg):
        self.get_logger().info(f"LM move_type: {msg.cmd_type}, move: {msg.move:3f} mm")
        move = msg.move
        with concurrent.futures.ThreadPoolExecutor() as executor:
            if msg.cmd_type == 'rel':
                future = executor.submit(self.lm_client.move_rel, move)
            elif msg.cmd_type == 'abs':
                future = executor.submit(self.lm_client.move_abs, move)


        '''if msg.cmd_type == 'rel':
            self.lm_client.move_rel(msg.move)
        elif msg.cmd_type == 'abs':
            self.lm_client.move_abs(msg.move)
        self.get_logger().info(f"LM move_type: {msg.cmd_type}, move: {msg.move:3f} mm")'''


def main(args=None):
    rclpy.init(args=args)

    node = LMControlNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
