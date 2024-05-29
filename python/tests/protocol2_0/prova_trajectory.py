import os
import time
from dynamixel_sdk import *  # Uses Dynamixel SDK library

# Control table address for MX-64
ADDR_MX_TORQUE_ENABLE = 64
ADDR_MX_GOAL_POSITION = 116
ADDR_MX_PRESENT_POSITION = 132

# Data Byte Length
LEN_MX_GOAL_POSITION = 4
LEN_MX_PRESENT_POSITION = 4

# Protocol version
PROTOCOL_VERSION = 2.0

# Default setting
DXL_ID = 1                   # Dynamixel ID
BAUDRATE = 57600             # Dynamixel default baudrate
DEVICENAME = '/dev/ttyUSB0'  # Check which port is being used on your controller
TORQUE_ENABLE = 1            # Value for enabling the torque
TORQUE_DISABLE = 0           # Value for disabling the torque

# Initialize PortHandler and PacketHandler instance
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

def read_goal_positions(filename):
    with open(filename, 'r') as file:
        lines = file.readlines()
        goal_positions = [(float(line.split()[0]), float(line.split()[1])) for line in lines]
    return goal_positions

def radian_to_dxl_position(radian):
    return int((radian / (2 * 3.141592653589793)) * 4095 + 2048)  # Conversion from radians to Dynamixel position value

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    quit()

# Enable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print(f"Failed to enable torque: {packetHandler.getTxRxResult(dxl_comm_result)}")
elif dxl_error != 0:
    print(f"Torque enable error: {packetHandler.getRxPacketError(dxl_error)}")
else:
    print("Dynamixel has been successfully connected")

# Read goal positions from file
goal_positions = read_goal_positions('posvel_sin0.2_dt16ms.txt')

# Inizializzare tempo iniziale
start_time = time.perf_counter()

# Set the goal positions at each timestep
for timestep, radian in goal_positions:
    dxl_position = radian_to_dxl_position(radian)
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_position)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Failed to write goal position: {packetHandler.getTxRxResult(dxl_comm_result)}")
    elif dxl_error != 0:
        print(f"Goal position write error: {packetHandler.getRxPacketError(dxl_error)}")
    #time.sleep(timestep)  # Wait for the specified timestep

    # Calcolare il tempo di attesa per il prossimo timestep
    next_time = start_time + timestep
    #while time.perf_counter() < next_time:
        #time.sleep(0.001)  # Sleep per un breve intervallo per ridurre il carico della CPU

# Disable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print(f"Failed to disable torque: {packetHandler.getTxRxResult(dxl_comm_result)}")
elif dxl_error != 0:
    print(f"Torque disable error: {packetHandler.getRxPacketError(dxl_error)}")

# Close port
portHandler.closePort()
