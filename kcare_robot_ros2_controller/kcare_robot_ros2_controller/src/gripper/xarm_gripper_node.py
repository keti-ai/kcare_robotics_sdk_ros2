import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

#For Robot tool Control
from xarm_msgs.srv import SetInt32, SetModbusTimeout, GetSetModbusData

from kcare_robot_ros2_controller_msgs.msg import GripperState
from kcare_robot_ros2_controller_msgs.srv import GripperCommand

import time

class RobotParam:
    spin_time: float = 0.05     # ROS2 루프문 대기시간

class XarmToolGripper(Node):
    def __init__(self):
        super().__init__('gripper_node')

                # Xarm Service clients
        SERVICE_CLIENTS ={
            'set_xarm_tool_baud' : ('/xarm/set_tgpio_modbus_baudrate',SetInt32),
            'set_xarm_tool_timeout' : ('/xarm/set_tgpio_modbus_timeout',SetModbusTimeout),
            'getset_xarm_modbus_data' : ('/xarm/getset_tgpio_modbus_data',GetSetModbusData),
        }
        
        self.service_clients = {}
        for service_tag, (service_name, service_type) in SERVICE_CLIENTS.items():
            self.service_clients[service_tag] = self.create_client(service_type, service_name)
            self.get_logger().info(f"Service Client created: {service_tag} -> {service_name}")
            

            
        self.srv_callback = MutuallyExclusiveCallbackGroup()
        self.gripper_service= self.create_service(GripperCommand,'gripper/command',self.set_gripperpose_callback,callback_group=self.srv_callback)
        
        

            

        
    def xarm_set_tool_baudrate(self,baudrate=115200):
        request=SetInt32.Request()
        request.data=baudrate

        future = self.service_clients['set_xarm_tool_baud'].call_async(request)
        while not future.done():
            # 여기서는 별도로 spin_once가 이미 다른 스레드에서 처리되므로
            # 추가적으로 spin_once(self)를 호출하지 않습니다.
            time.sleep(RobotParam.spin_time)
            
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Xarm set Tool Baud : {response.message}')
        else:
            self.get_logger().error('Failed to call service /xarm/set_tgpio_modbus_baudrate') 
            

    def xarm_set_tool_timeout(self,timeout=30):
        request=SetModbusTimeout.Request()
        request.timeout=timeout

        future = self.service_clients['set_xarm_tool_timeout'].call_async(request)
        while not future.done():
            # 여기서는 별도로 spin_once가 이미 다른 스레드에서 처리되므로
            # 추가적으로 spin_once(self)를 호출하지 않습니다.
            time.sleep(RobotParam.spin_time)
            
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Xarm set Tool Timeout : {response.message}')
        else:
            self.get_logger().error('Failed to call service /xarm/set_tgpio_modbus_timeout') 
        
        
    def xarm_modbus_data(self,data,wait=True):
        request=GetSetModbusData.Request()
        request.modbus_data=data
        request.modbus_length=len(data)
        request.ret_length=11
        
        future = self.service_clients['getset_xarm_modbus_data'].call_async(request)
        
        if not wait:
            return None
        
        while not future.done():
            # 여기서는 별도로 spin_once가 이미 다른 스레드에서 처리되므로
            # 추가적으로 spin_once(self)를 호출하지 않습니다.
            time.sleep(RobotParam.spin_time)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Xarm Send Modbus Result Return : {response.ret_data}')
            return response.ret_data
        else:
            self.get_logger().error('Failed to call service getset_xarm_modbus_data')
            return None
        
    def xarm_gripper_init(self):
        data = [0x01, 0x10, 0x00, 0x00, 0x00, 0x02, 0x04, 0x00, 0x65, 0x00, 0x00]
        return self.xarm_modbus_data(data)
        
    def xarm_set_motor_torque(self, ratio):
        data = [
            0x01,
            0x10,
            0x00, 0x00,
            0x00, 0x02,
            0x04,
            0x00, 0xD4,  # Command 212
            (ratio >> 8) & 0xFF, ratio & 0xFF
        ]
        return self.xarm_modbus_data(data)

    def xarm_set_finger_position(self, position):
        data = [
            0x01,       # slave ID
            0x10,       # Function code: Write Multiple Registers
            0x00, 0x00, # Start address: Register 0
            0x00, 0x02, # Register count: 2
            0x04,       # Byte count: 4 bytes (2 registers)
            0x00, 0x68, # Register 0: Command 104 (Set Finger Position)
            (position >> 8) & 0xFF, position & 0xFF  # Register 1: Position
        ]
        return self.xarm_modbus_data(data,wait=False)

    def xarm_set_gripper_speed_percent(self, percent):
         data = [
             0x01,       # slave ID
             0x10,       # function code: Write Multiple Registers
             0x00, 0x00, # start address: register 0
             0x00, 0x02, # number of registers to write
             0x04,       # byte count (2 regs × 2 bytes)
             0x00, 0xD5, # command 213 = Set Speed (as percent)
             (percent >> 8) & 0xFF, percent & 0xFF  # parameter
         ]
         return self.xarm_modbus_data(data)
 
    def xarm_enable_impedance(self):
        data = [
            0x01,       # slave ID
            0x10,       # function code: Write Multiple Registers
            0x00, 0x00, # start address register 0
            0x00, 0x02, # number of registers = 2
            0x04,       # byte count = 4 bytes
            0x00, 0x6C, # command 108 (Enable Impedance Control)
            0x00, 0x00  # no parameter
        ]
        return self.xarm_modbus_data(data)

    def xarm_set_impedance_parameters(self, level):
        if level < 1 or level > 10:
            raise ValueError("Impedance level must be between 1 and 10")

        data = [
            0x01,       # slave ID
            0x10,       # function code: Write Multiple Registers
            0x00, 0x00, # start address
            0x00, 0x03, # register count = 3 (command + 2 values)
            0x06,       # byte count = 6 bytes (3 * 2)
            0x00, 0x6E, # 110 decimal = 0x6E (Set Impedance Parameters)
            0x00, 0x03, # Value 1: 3 (Gripper 종류)
            0x00, level # Value 2: 1~10 범위
        ]
        return self.xarm_modbus_data(data)


    def rb_init(self):
        self.xarm_set_tool_baudrate(baudrate=115200)
        # 툴 타임아웃 세팅
        self.xarm_set_tool_timeout(timeout=30)
        # 그리퍼 초기화
        self.xarm_enable_impedance()
        self.xarm_gripper_init()
        self.xarm_set_gripper_speed_percent(100)


    def set_gripperpose_callback(self,request,response):
        self.get_logger().info(f"Gripper Service Call.")
        if (0 <= request.pose <= 1000 and 1<= request.force <= 10):
            self.xarm_set_impedance_parameters(request.force)
            time.sleep(0.05)
            self.xarm_set_finger_position(request.pose)
            self.get_logger().info(f"Gripper Work Fine. Pose : {request.pose}, Force : {request.force}.")
            response.successed=True
        else:
            self.get_logger().info(f"Gripper request out of range.")
            response.successed=False
        return response



def main(args=None):
    rclpy.init(args=args)
    master_node = XarmToolGripper()
    
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(master_node)
    # 노드내 함수 비동기 실행.초기화 함수등 실행
    time.sleep(0.5)
    executor.create_task(master_node.rb_init)
    try:
        master_node.get_logger().info("✅ Gripper Server is running...")
        executor.spin()  # ✅ 멀티스레드 실행
    except KeyboardInterrupt:
        pass
    finally:
        master_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()