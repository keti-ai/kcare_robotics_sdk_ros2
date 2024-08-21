import dynamixel_sdk as dxl
import time

# Control table addresses and constants
ADDR_GOAL_VELOCITY = 104
DXL_ID = 1
PROTOCOL_VERSION = 2.0
BAUDRATE = 115200
DEVICENAME = '/dev/ttyHead'

# Initialize PortHandler instance
portHandler = dxl.PortHandler(DEVICENAME)
packetHandler = dxl.PacketHandler(PROTOCOL_VERSION)

# Open port and set baudrate
portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)

# Enable Dynamixel Torque
TORQUE_ENABLE = 1
ADDR_TORQUE_ENABLE = 64
packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

# Set a goal velocity within the allowed range
goal_velocity = 10  # This should be within the allowed range for most Dynamixel models

# Write goal velocity
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_VELOCITY, goal_velocity)
if dxl_comm_result != dxl.COMM_SUCCESS:
    print(f"Failed to set goal velocity: {packetHandler.getTxRxResult(dxl_comm_result)}")
elif dxl_error != 0:
    print(f"Error occurred: {packetHandler.getRxPacketError(dxl_error)}")
else:
    print("Goal velocity set.")
time.sleep(10)
