from pymodbus.client import ModbusSerialClient
import struct


class MD485DriverWrapper:
    motor_id = 1

    def __init__(self, port, baud):
        self.port = port
        self.baud = baud

        self.client = ModbusSerialClient(self.port, baudrate=self.baud)

    def connect_lm(self):
        # 포트 열기
        self.client.connect()

    def disconnect_lm(self):
        # 포트 닫기
        self.client.close()

    def set_velocity(self,rpm):
        address = 130
        rpm = int(rpm) & 0xFFFF
        self.client.write_register(address,rpm,slave=self.motor_id)

    def set_position_with_velocity(self,position, max_speed):
        address = 219
        pos_bytes = struct.pack('<i', position)  # 4-byte signed int (little-endian)
        vel_bytes = struct.pack('<H', max_speed)  # 2-byte unsigned int
        values = list(pos_bytes + vel_bytes)
        registers = struct.unpack('<3H', bytes(values))
        self.client.write_registers(address, registers, slave=self.motor_id)

    def inc_position_with_velocity(self, delta_pos, max_speed):
        """
        현재 위치에서 delta_pos만큼 이동, max_speed는 rpm 단위
        PID = 220 (2 words + 1 word → 총 3개 = 6 바이트)
        """
        address = 220
        pos_bytes = struct.pack('<i', delta_pos)       # 4 bytes
        speed_bytes = struct.pack('<H', max_speed)     # 2 bytes

        data = list(pos_bytes + speed_bytes)
        registers = struct.unpack('<3H', bytes(data))  # 3 words

        self.client.write_registers(address, registers, slave=self.motor_id)
        print(f"Incremental move: Δ{delta_pos} @ {max_speed}rpm")


    def reset_pose(self):
        PID_COMMAND = 10
        CMD_POSI_RESET = 10
        self.client.write_register(PID_COMMAND, CMD_POSI_RESET, slave=self.motor_id)

    def set_in_position_resolution(self,resolution=50):
        """
        위치 해상도 설정 (PID 171)
        :param resolution: 허용 오차 값 (int)
        """
        self.client.write_register(171, resolution, slave=self.motor_id)
        print(f"In Position resolution set to ±{resolution}")


    def read_monitor(self):
        result = self.client.read_holding_registers(196, count=6, slave=self.motor_id)
        if result.isError():
            print("Error reading monitor data")
            return

        registers = result.registers
        speed = struct.unpack('<h', struct.pack('<H', registers[0]))[0]
        current = struct.unpack('<h', struct.pack('<H', registers[1]))[0] / 10
        output = struct.unpack('<h', struct.pack('<H', registers[2]))[0]
        position = struct.unpack('<i', struct.pack('<HH', registers[3], registers[4]))[0]
        status1 = registers[5] & 0xFF
        status2 = (registers[5] >> 8) & 0xFF

        # print("=== Monitor ===")
        # print(f"Speed    : {speed} rpm")
        # print(f"Current  : {current:.1f} A")
        # print(f"Output   : {output}")
        # print(f"Position : {position}")
        # print(f"Status1  : {bin(status1)}")
        # print(f"Status2  : {bin(status2)}")
        # print("================")

        return [speed,current,position]

    def read_io_status(self):
        result = self.client.read_holding_registers(48, count=1, slave=self.motor_id)
        if result.isError():
            print("Error reading I/O status")
            return

        di_status = result.registers[0]
        dir_input = (di_status >> 2) & 0x01
        run_brake_input = (di_status >> 3) & 0x01
        start_stop_input = (di_status >> 4) & 0x01

        # print("=== I/O Status ===")
        # print(f"DIR Input        : {dir_input}")
        # print(f"RUN/BRAKE Input  : {run_brake_input}")
        # print(f"START/STOP Input : {start_stop_input}")
        # print(f"Raw              : {bin(di_status)}")
        # print("===================")

        return [dir_input,run_brake_input,start_stop_input]

    def read_target_position(self):
        result = self.client.read_holding_registers(230, count=2, slave=self.motor_id)
        if result.isError():
            print("Error reading target position")
            return None

        regs = result.registers
        target_position = struct.unpack('<i', struct.pack('<HH', regs[0], regs[1]))[0]
        #print(f"Target Position: {target_position}")
        return target_position


    def read_moving_status(self):
        result = self.client.read_holding_registers(49, count=1, slave=self.motor_id)
        if result.isError():
            print("Error reading movement status")
            return

        in_position = result.registers[0]
        moving = (in_position == 0)

        print("=== Moving Status ===")
        print(f"In Position : {not moving} (1: Yes, 0: No)")
        print(f"Moving      : {moving} (1: Moving, 0: Stopped)")
        print("======================")

        return in_position

    def count_to_mm(self, count):
        milmeter = (count / 56000.0) * 10.0
        return milmeter

    def mm_to_count(self, milmeter):
        count = (milmeter / 10.0) * 56000
        return count


    def read_motor_status(self):
        self.read_monitor()
        self.read_io_status()
        self.read_moving_status()



import time

if __name__ == '__main__':
    mddrv_ = MD485DriverWrapper('/dev/ttyACM0',19200)
    mddrv_.connect_lm()

    mddrv_.reset_pose()
    mddrv_.set_in_position_resolution()
    mddrv_.read_motor_status()

    #mddrv_.set_position_with_velocity(560000,3000)
    mddrv_.set_velocity(-2000)
    while True:
        mddrv_.read_io_status()
        time.sleep(1)


    mddrv_.disconnect_lm()