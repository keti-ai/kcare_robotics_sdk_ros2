import time
import dynamixel_sdk as dxl_sdk
from kcare_robot_ros2_controller.src.head.dxl_utils.u2d2 import U2D2


def DXL_ADDR():
    config = dict()
    config["ADDR_OPERATING_MODE"] = 11
    config["ADDR_TORQUE_ENABLE"] = 64
    config["ADDR_GOAL_POSITION"] = 116
    config["ADDR_GOAL_VELOCITY"] = 104
    config["ADDR_PRESENT_POSITION"] = 132
    config["ADDR_PRESENT_VELOCITY"] = 128
    config["ADDR_PROFILE_VELOCITY"] = 112
    config["ADDR_PROFILE_ACCELERATION"] = 108
    config["ADDR_VELOCITY_I_GAIN"] = 76
    config["ADDR_VELOCITY_P_GAIN"] = 78
    config["ADDR_POSITION_D_GAIN"] = 80
    config["ADDR_POSITION_I_GAIN"] = 82
    config["ADDR_POSITION_P_GAIN"] = 84
    config["DXL_MINIMUM_POSITION_VALUE"] = -150000
    config["DXL_MAXIMUM_POSITION_VALUE"] =  150000

    return config


class Dynamixel:
    def __init__(self, logger):
        self.logger = logger
        self.dxl_addr = DXL_ADDR()
        
        self.u2d2 = None

        self.dxl_ids = []
        self.dxl_count = 0
        self.dxl_goal_pose = []
        self.dxl_curr_pose = []
        self.dxl_home = []
        self.dxl_limit = []
        self.dxl_speed = []
        self.dxl_offset = []
        self.dxl_enable = False
        
        self.control_mode = ""
        
    def set(self, baudrate, protocol, device_name, device_ids, device_home,
            device_limit, device_speed, device_offset):
        self.u2d2 = U2D2(self.logger)
        self.u2d2.set(baudrate, protocol, device_name)
        
        self.dxl_ids = device_ids
        self.dxl_count = len(self.dxl_ids)
        self.dxl_goal_pose = [0 for i in range(self.dxl_count)]
        self.dxl_curr_pose = [0 for i in range(self.dxl_count)]
        self.dxl_home = device_home
        self.dxl_limit = device_limit
        self.dxl_speed = device_speed
        self.dxl_offset = device_offset

    def init(self):
        self.u2d2.init()
        self.logger.info("Complete U2D2 connection.")

        self.set_mode('velocity')

    def close(self):
        self.disable()

        self.u2d2.close()
    
    def set_mode(self, mode:str='position'):
        
        VELOCITY_CONTROL_MODE = 1
        POSITION_CONTROL_MODE = 3

        mode = mode.lower()
        if mode in ['position', 'velocity']:
            self.disable()
                            
            if mode == 'position':
                
                self.u2d2.packet_handler.write1ByteTxRx(self.u2d2.port_handler,
                                                        1,
                                                        self.dxl_addr["ADDR_OPERATING_MODE"],
                                                        POSITION_CONTROL_MODE)
                self.u2d2.packet_handler.write1ByteTxRx(self.u2d2.port_handler,
                                                        2,
                                                        self.dxl_addr["ADDR_OPERATING_MODE"],
                                                        POSITION_CONTROL_MODE)
                self.control_mode = 'position'
                
                self.enable()
                
                self.set_speed(1)
                self.set_speed(2)
                
                # self.logger.info("Head control mode changed to ['position']")
                
            elif mode == 'velocity':
                self.u2d2.packet_handler.write1ByteTxRx(self.u2d2.port_handler,
                                                        1,
                                                        self.dxl_addr["ADDR_OPERATING_MODE"],
                                                        VELOCITY_CONTROL_MODE)
                self.u2d2.packet_handler.write1ByteTxRx(self.u2d2.port_handler,
                                                        2,
                                                        self.dxl_addr["ADDR_OPERATING_MODE"],
                                                        VELOCITY_CONTROL_MODE)
                self.control_mode = 'velocity'
                
                self.enable()

                # self.logger.info("Head control mode changed to ['velocity']")
            
        else:
            self.logger.error("Wrong head control mode. Choose in ['position', 'velocity']")

    def set_speed(self, id):
        self.u2d2.packet_handler.write4ByteTxRx(self.u2d2.port_handler,
                                                id,
                                                self.dxl_addr["ADDR_PROFILE_VELOCITY"],
                                                self.dxl_speed[id - 1])

    def set_gain(self, id):
        gains = self.dxl_config[f'id{id}']

        pid_gain_list = ["VELOCITY_I_GAIN", "VELOCITY_P_GAIN", "POSITION_D_GAIN", "POSITION_I_GAIN", "POSITION_P_GAIN"]

        for pid_gain in pid_gain_list:
            addr_gain = self.dxl_addr[f"ADDR_{pid_gain}"]
            conn_result, conn_error = self.u2d2.packet_handler.write4ByteTxRx(self.u2d2.port_handler,
                                                                        id,
                                                                        addr_gain,
                                                                        gains[pid_gain])

    def enable(self):
        if not self.dxl_enable:
            for id in self.dxl_ids:
                conn_result, conn_error = self.u2d2.packet_handler.write1ByteTxRx(self.u2d2.port_handler,
                                                                                id,
                                                                                self.dxl_addr["ADDR_TORQUE_ENABLE"],
                                                                                True)
            self.dxl_enable = True

    def disable(self):
        if self.dxl_enable:
            for id in self.dxl_ids:
                conn_result, conn_error = self.u2d2.packet_handler.write1ByteTxRx(self.u2d2.port_handler,
                                                                                id,
                                                                                self.dxl_addr["ADDR_TORQUE_ENABLE"],
                                                                                False)
            self.dxl_enable = False

    def position_write(self, id, goal_position):
        if not self.control_mode == "position":
            self.set_mode("position")
        conn_result, conn_error = self.u2d2.packet_handler.write4ByteTxRx(self.u2d2.port_handler,
                                                                          id,
                                                                          self.dxl_addr["ADDR_GOAL_POSITION"],
                                                                          goal_position)
        self.dxl_goal_pose[id - 1] = goal_position
        
    def velocity_write(self, id, goal_velocity):
        if not self.control_mode == "velocity":
            self.set_mode("velocity")

        conn_result, conn_error = self.u2d2.packet_handler.write4ByteTxRx(self.u2d2.port_handler,
                                                                          id,
                                                                          self.dxl_addr["ADDR_GOAL_VELOCITY"],
                                                                          goal_velocity)
            
    def pose(self, id):
        try:
            curr_position, conn_result, conn_error = self.u2d2.packet_handler.read4ByteTxRx(self.u2d2.port_handler,
                                                                                            id,
                                                                                            self.dxl_addr["ADDR_PRESENT_POSITION"])
        except:
            curr_position = self.pose(id)
        
        if curr_position == 0:
            if len(self.dxl_curr_pose) == 2:
                curr_position = self.dxl_curr_pose[id - 1]
        else:
            self.dxl_curr_pose[id - 1] = curr_position

        return curr_position
    
    def speed(self, id):
        curr_velocity, conn_result, conn_error = self.u2d2.packet_handler.read4ByteTxRx(self.u2d2.port_handler,
                                                                                        id,
                                                                                        self.dxl_addr["ADDR_PRESENT_VELOCITY"])
        if curr_velocity > 1023:
            curr_velocity -= 4096
            
        return curr_velocity
        
    def go_home(self):
        while True:
            self.position_write(1, self.dxl_home[0])
            self.position_write(2, self.dxl_home[1])

            offset = 50     # encoder error offset

            if (self.dxl_home[0] - offset < self.pose(1) < self.dxl_home[0] + offset) and \
                (self.dxl_home[1] - offset < self.pose(2) < self.dxl_home[1] + offset):
                self.position_write(1, self.dxl_home[0])
                self.position_write(2, self.dxl_home[1])
                
                break

        self.logger.info("Head module arrived home")
            
    def position_control(self, rz, ry):
        if self.safety(id=1,value=rz):
            #rz += self.pose(id=1)
            self.position_write(id=1, goal_position=self.deg2encoder(rz) + self.deg2encoder(self.dxl_offset[0]))
        # else:
        #     self.position_write(id=1, goal_velocity=0)
            
        if self.safety(id=2,value=ry):
            #ry += self.pose(id=2)
            self.position_write(id=2, goal_position=self.deg2encoder(ry) + self.deg2encoder(self.dxl_offset[1]))
        # else:
        #     self.position_write(id=2, goal_velocity=0)

    def velocity_control(self, rz, ry):
        if self.safety(id=1, value=rz):
            self.velocity_write(id=1, goal_velocity=rz)
        else:
            self.velocity_write(id=1, goal_velocity=0)
            
        if self.safety(id=2, value=ry):
            self.velocity_write(id=2, goal_velocity=ry)
        else:
            self.velocity_write(id=2, goal_velocity=0)
            
    def safety(self, id, value):
        safe = False

        if self.dxl_limit[id - 1][0] < self.pose(id) < self.dxl_limit[id - 1][1]:
            safe = True
        else:
            if self.dxl_limit[id - 1][0] > self.pose(id):
                if self.control_mode == 'velocity':
                    if value > 0:
                        safe = True
            elif self.dxl_limit[id - 1][1] < self.pose(id):
                if self.control_mode == 'velocity':
                    if value < 0:
                        safe = True
        return safe
    
    def get_pose(self, id):
        # return self.encoder2deg(self.pose(id)) - self.dxl_offset[id - 1]
        return self.dxl_offset[id - 1] - self.encoder2deg(self.pose(id))
    
    @staticmethod
    def deg2encoder(deg):
        encoder = deg * (4096 / 360)
        return int(encoder)

    @staticmethod
    def encoder2deg(encoder):
        deg = encoder * (360 / 4096)
        return deg

        