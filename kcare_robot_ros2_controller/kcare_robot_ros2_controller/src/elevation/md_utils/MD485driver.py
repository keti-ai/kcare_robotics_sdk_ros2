from pymodbus.client import ModbusSerialClient
import struct


class MD485DriverWrapper:
    def __init__(self, ros_node=None, debug=False, **kwargs):
        self.ros_node = ros_node
        self.debug = debug

        self.port = kwargs.get('port','/dev/ttyLM')
        self.baud = kwargs.get('baudrate',19200)

        #self.home_pos = kwargs.get('home_pos', 110)              # 홈 위치 (기본값 0)
        self.elevation_range = kwargs.get('elevation_range', [0.0, 600.0])
        self.offset_position = kwargs.get('offset_position', 0.0)
        self.encoder_ppr = kwargs.get('encoder_ppr', 1000)     # 엔코더 해상도 PPR
        self.reduction_ratio = kwargs.get('reduction_ratio', 14) # 감속비
        self.lead_pitch = kwargs.get('lead_pitch', 10) # 리드 피치 (mm)
        self.rpm_range = kwargs.get('rpm_range', [-5000, 5000]) # 속도(RPM) 범위
        self.limit_inveted = kwargs.get('limit_inveted', False) # 리미트센서 반전여부
        self.motor_inv = kwargs.get('motor_inv',False)
        self.motor_id = kwargs.get('motor_id', 1) # 모터드라이버 모드버스 슬레이브 ID (기본 1)


        self.client = ModbusSerialClient(self.port, baudrate=self.baud)


        self.initialized = False
        self.current_position = 0.0
        self.target_position = 0.0
        self.emergency_stop = False
        self.limit_status = False
        self.moving = False

    def log_info(self, msg):
        if self.ros_node:
            self.ros_node.get_logger().info(msg)
        else:
            print(f"[INFO] {msg}")

    def log_warning(self, msg):
        if self.ros_node:
            self.ros_node.get_logger().warn(msg)  # ROS2 warn
        else:
            print(f"[WARNING] {msg}")

    def log_error(self, msg):
        if self.ros_node:
            self.ros_node.get_logger().error(msg)
        else:
            print(f"[ERROR] {msg}")

    def log_debug(self, msg):
        # debug 모드가 False면 무시
        if not self.debug:
            return

        if self.ros_node:
            self.ros_node.get_logger().debug(msg)
        else:
            print(f"[DEBUG] {msg}")  # 비ROS 환경에서는 print


    def connect_lm(self):
        # 포트 열기
        self.client.connect()
        self.set_inv_velocity(self.motor_inv)

    def disconnect_lm(self):
        # 포트 닫기
        self.client.close()

    def set_velocity_rpm(self,rpm):
        address = 130
        rpm = max(min(rpm, self.rpm_range[1]), self.rpm_range[0])
        rpm = int(rpm) & 0xFFFF
        self.client.write_register(address,rpm,slave=self.motor_id)

    def set_linear_velocity(self,mm):
        rpm=self.mm_per_s_to_rpm(mm)
        self.set_velocity_rpm(rpm)

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
        self.log_info(f"Incremental move: Δ{delta_pos} @ {max_speed}rpm")

    def move_inc_pose_mm(self,mm,linear_speed):
        count,rpm = int(self.mm_to_count(mm)), int(self.mm_per_s_to_rpm(linear_speed))
        self.inc_position_with_velocity(count,rpm)

    def move_abs_pose_mm(self,mm,linear_speed):
        if mm < self.elevation_range[0]:
            self.log_warning(f"elevation {mm} below minimum {self.elevation_range[0]}, clamped to {self.elevation_range[0]}")
            mm = self.elevation_range[0]
        elif mm > self.elevation_range[1]:
            self.log_warning(f"elevation {mm} above maximum {self.elevation_range[1]}, clamped to {self.elevation_range[1]}")
            mm = self.elevation_range[1]
        count,rpm = int(self.mm_to_count(mm-self.offset_position)), int(self.mm_per_s_to_rpm(linear_speed))

        self.set_position_with_velocity(count,rpm)

    def reset_pose(self):
        PID_COMMAND = 10
        CMD_POSI_RESET = 10
        self.client.write_register(PID_COMMAND, CMD_POSI_RESET, slave=self.motor_id)


    def read_monitor(self):
        result = self.client.read_holding_registers(196, count=6, slave=self.motor_id)
        if result.isError():
            self.log_error("Reading monitor data")
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
            self.log_error("Reading I/O status")
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
    

    
    def get_emergency_stop(self):
        io_status = self.read_io_status()
        if io_status is None:
            return None
        emergency_stop = io_status[1]
        return emergency_stop


    def count_to_mm(self, count):
        mm = (count / (self.encoder_ppr * 4)) / self.reduction_ratio * self.lead_pitch
        return mm

    def mm_to_count(self, millimeter):
        output_rev = millimeter / self.lead_pitch
        motor_rev = output_rev * self.reduction_ratio
        count = motor_rev * self.encoder_ppr * 4
        return int(count)

    def rpm_to_mm_per_s(self, rpm):
        motor_rev_per_s = rpm / 60.0
        output_rev_per_s = motor_rev_per_s / self.reduction_ratio
        linear_velocity = output_rev_per_s * self.lead_pitch
        return linear_velocity  # mm/s

    def mm_per_s_to_rpm(self, linear_velocity):
        output_rev_per_s = linear_velocity / self.lead_pitch
        motor_rev_per_s = output_rev_per_s * self.reduction_ratio
        rpm = motor_rev_per_s * 60.0
        return rpm


    def read_current_position(self):
        self.current_position = self.count_to_mm(self.read_monitor()[2]) + self.offset_position
        self.log_debug(self.current_position)
        return self.current_position

    def read_target_position(self):
        result = self.client.read_holding_registers(230, count=2, slave=self.motor_id)
        if result.isError():
            self.log_error(f"Reading target position")
            return None
        
        regs = result.registers
        target_count = struct.unpack('<i', struct.pack('<HH', regs[0], regs[1]))[0]
        self.target_position = self.count_to_mm(target_count) + self.offset_position
        self.log_debug(self.target_position)

        return self.target_position


    def get_lower_limit_switch(self):
        io_status = self.read_io_status()
        if io_status is None:
            return None

        # limit_inverted 여부에 따라 읽는 인덱스 선택
        if not self.limit_inveted: 
            self.limit_status = io_status[2]
        else:
            self.limit_status = io_status[0]

        self.log_debug(self.limit_status)
        return self.limit_status

    
    def get_emergency_stop(self):
        io_status = self.read_io_status()
        if io_status is None:
            return None
        emergency_stop = io_status[1]
        return emergency_stop


    def read_in_position_status(self):
        result = self.client.read_holding_registers(49, count=1, slave=self.motor_id)
        if result.isError():
            self.log_error(f"Reading in position status")
            return

        in_position = result.registers[0]

        self.log_debug(f"In position status {in_position}")

        return in_position


    def set_inv_velocity(self,direction):
        PID_COMMAND=16
        self.client.write_register(PID_COMMAND, direction, slave=self.motor_id)

    def read_init_set_ok(self):
        PID_COMMAND=66
        result = self.client.read_holding_registers(PID_COMMAND, count=1, slave=self.motor_id)
        return result.registers[0]

    def set_init_set(self,data):
        '''Warning PID 66 is PID_TQ_RATIO. We use this PID to set init_set flag'''
        PID_COMMAND=66
        self.client.write_register(PID_COMMAND, data, slave=self.motor_id)

    ## BLDC 모터 위치결정도 mm. 모터가 이동후에도 이동완료로 안되면 값 조절
    def set_in_position_resolution(self,in_position_mm):
        count=int(self.mm_to_count(in_position_mm))
        self.log_debug(count)
        PID_COMMAND=171
        self.client.write_register(PID_COMMAND, count, slave=self.motor_id)


if __name__ == '__main__':
    import time

    mddrv_ = MD485DriverWrapper(
        debug=True,
        port='/dev/ttyUSB0',
        baudrate=19200,
        elevation_range=[110.0,700.0],
        offset_position=112.0,
        encoder_ppr=16384,
        reduction_ratio=28/19,
        rpm_range=[-3000, 3000],
        limit_inveted=True,
        motor_inv=True
        )
    #mddrv_ = MD485DriverWrapper(port='/dev/ttyUSB0',baudrate=19200)
    
    mddrv_.connect_lm()



    mddrv_.set_linear_velocity(-20.0)

    time.sleep(3)


    mddrv_.set_linear_velocity(0.0)


    #mddrv_.set_in_position_resolution(0.5)

    #mddrv_.move_abs_pose_mm(150.0,100.0)
    #time.sleep(0.01)
    #mddrv_.read_in_position_status()

    #mddrv_.set_velocity_rpm(0)
    #mddrv_.set_linear_velocity(0)
    # while True:
    #     mddrv_.move_abs_pose_mm(110.0,150.0)
    #     mddrv_.read_target_position()
    #     time.sleep(1.5)
    #     mddrv_.move_abs_pose_mm(150.0,150.0)
    #     mddrv_.read_target_position()
    #     time.sleep(1.5)

    #mddrv_.move_inc_pose_mm(10.0,100.0)

    #mddrv_.set_init_set(0)

    #print(mddrv_.read_init_set_ok())

    mddrv_.disconnect_lm()