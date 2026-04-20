from dynamixel_sdk.group_sync_read import GroupSyncRead
from dynamixel_sdk.group_sync_write import GroupSyncWrite
from dynamixel_sdk.packet_handler import PacketHandler
from dynamixel_sdk.port_handler import PortHandler
from dynamixel_sdk.robotis_def import (
    COMM_SUCCESS,
    DXL_HIBYTE,
    DXL_HIWORD,
    DXL_LOBYTE,
    DXL_LOWORD,
)

from typing import Sequence
import numpy as np

# # 조인트 각도 범위 (예시)
# JOINT_LIMITS_DEG = {
#     0: (90-50, 90+35),  # Joint 0: min, max
#     1: (0-5, 0+110),  # Joint 1: min, max
#     2: (0-90, 0+90),  # Joint 2: min, max
#     3: (0, 0+135),  # Joint 3: min, max
#     4: (0-100, 0+100),  # Joint 4: min, max
#     5: (0-92, 0+95),  # Joint 5: min, max
#     6: (0-135, 0+135),  # Joint 6: min, max
#     7: (-30, 0) # Gripper min, max
# }




# Constants
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
LEN_GOAL_POSITION = 4
ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
ADDR_GOAL_CURRENT = 102
LEN_GOAL_CURRENT = 2


class RemoteDynamixelWrapper:
    def __init__(self, port_name, baudrate, ids):
        # 포트 및 시리얼 통신 설정
        self._port_handler = PortHandler(port_name)
        self._packet_handler = PacketHandler(2.0)  # 2.0은 프로토콜 버전 2.0을 의미합니다.

        # 포트 열기
        if not self._port_handler.openPort():
            raise Exception("Failed to open port.")
        if not self._port_handler.setBaudRate(baudrate):
            raise Exception("Failed to set baud rate.")

        self.device_ids = ids

        self._group_sync_read = GroupSyncRead(self._port_handler, self._packet_handler, ADDR_PRESENT_POSITION,
                                                 LEN_PRESENT_POSITION)
        self._group_sync_write = GroupSyncWrite(self._port_handler, self._packet_handler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

        for dxl_id in self.device_ids:
            if not self._group_sync_read.addParam(dxl_id):
                raise RuntimeError(
                    f"Failed to add parameter for Dynamixel with ID {dxl_id}"
                )

        # Disable torque for each Dynamixel servo
        self._torque_enabled = False
        try:
            self.set_torque_mode(self._torque_enabled)
        except Exception as e:
            print(f"port: {port_name}, {e}")

    def torque_enabled(self) -> bool:
        return self._torque_enabled

    def set_torque_mode(self, enable: bool):
        torque_value = TORQUE_ENABLE if enable else TORQUE_DISABLE
        for dxl_id in self.device_ids:
            dxl_comm_result, dxl_error = self._packet_handler.write1ByteTxRx(
                self._port_handler, dxl_id, ADDR_TORQUE_ENABLE, torque_value
            )
            if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                print(dxl_comm_result)
                print(dxl_error)
                raise RuntimeError(
                    f"Failed to set torque mode for Dynamixel with ID {dxl_id}"
                )

        self._torque_enabled = enable

    def set_target_current(self, currents):
        for dxl_id in self.device_ids:
            dxl_comm_result, dxl_error = self._packet_handler.write2ByteTxRx(
                self._port_handler, dxl_id, ADDR_GOAL_CURRENT, currents
            )
            if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                print(dxl_comm_result)
                print(dxl_error)
                raise RuntimeError(
                    f"Failed to set torque mode for Dynamixel with ID {dxl_id}"
                )
            


    def set_target_joint(self,  joint_angles: Sequence[float]):
        if len(joint_angles) != len(self.device_ids):
            raise ValueError(
                "The length of joint_angles must match the number of servos"
            )
        if not self._torque_enabled:
            raise RuntimeError("Torque must be enabled to set joint angles")

        for dxl_id, angle in zip(self.device_ids, joint_angles):
            # Convert the angle to the appropriate value for the servo
            position_value = int(angle * 2048 / np.pi)

            # Allocate goal position value into byte array
            param_goal_position = [
                DXL_LOBYTE(DXL_LOWORD(position_value)),
                DXL_HIBYTE(DXL_LOWORD(position_value)),
                DXL_LOBYTE(DXL_HIWORD(position_value)),
                DXL_HIBYTE(DXL_HIWORD(position_value)),
            ]

            # Add goal position value to the Syncwrite parameter storage
            dxl_addparam_result = self._group_sync_write.addParam(
                dxl_id, param_goal_position
            )
            if not dxl_addparam_result:
                raise RuntimeError(
                    f"Failed to set joint angle for Dynamixel with ID {dxl_id}"
                )

        # Syncwrite goal position
        dxl_comm_result = self._group_sync_write.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            raise RuntimeError("Failed to syncwrite goal position")

        # Clear syncwrite parameter storage
        self._group_sync_write.clearParam()

    def read_present_positions(self):
        if self._group_sync_read.txRxPacket() != COMM_SUCCESS:
            raise Exception("Failed to read present positions.")

        positions = {}
        for device_id in self.device_ids:
            data = self._group_sync_read.getData(device_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            positions[device_id]= np.int32(np.uint32(data)) / 2048.0 * np.pi

        return positions

    def read_only_joints(self):
        arra=self.read_present_positions()
        values = [v for k, v in arra.items() if k != 8]
        return np.array(values), arra[8]

    def read_joints(self):
        return np.array(list(self.read_present_positions().values()))

    def close(self):
        self._port_handler.closePort()



if __name__ == '__main__':
    import time

    driver = RemoteDynamixelWrapper('/dev/ttyUSB0', 57600, [1, 2, 3, 4, 5, 6, 7, 8])
    driver.set_target_current(30)
    driver.set_torque_mode(True)
    driver.set_target_joint([1.57, 0.0, 0.0, 0.0, 0.0, -1.57, 0.0, 0.0])

    while True:
        print(driver.read_only_joints())
        time.sleep(0.05)