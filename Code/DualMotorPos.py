import time
import numpy as np
import os
from dynamixel_sdk import *                    # Uses Dynamixel SDK library

# Control table address
ADDR_PRO_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_POSITION   = 132

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL1_ID                     = 10                 # Dynamixel#1 ID : 1
DXL2_ID                     = 11                 # Dynamixel#2 ID : 2
BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                 = 'COM10'    # Check which port is being used on your controller
                                         # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = -3000           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 3000            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold


def cal_position1(current_time):
    # Get the current time in seconds

    # Adjust the frequency by multiplying time by a scaling factor (2*pi) for 10-second frequency
    frequency = 1.0 / 10.0  # 10 seconds per cycle
    angle = 2 * np.pi * frequency * current_time

    amplitude = 3000  # Adjust this value to control the amplitude of the sine wave
    offset = 0  # Adjust this value to control the vertical position of the sine wave

    pos = amplitude * np.sin(angle) + offset

    # Optional: Clamp the position within a certain range
    pos_max = 3000
    pos_min = -3000

    if pos > pos_max:
        pos = pos_max
    if pos < pos_min:
        pos = pos_min
    print("position 1: ")
    print(pos)
    return pos

def cal_position2(current_time):

    # Adjust the frequency by multiplying time by a scaling factor (2*pi) for 10-second frequency
    frequency = 1.0 / 10.0  # 10 seconds per cycle
    angle = 2 * np.pi * frequency * current_time

    amplitude = 3000  # Adjust this value to control the amplitude of the sine wave
    offset = 0  # Adjust this value to control the vertical position of the sine wave

    pos = amplitude * np.sin(angle+np.pi/2) + offset

    # Optional: Clamp the position within a certain range
    pos_max = 3000
    pos_min = -3000

    if pos > pos_max:
        pos = pos_max
    if pos < pos_min:
        pos = pos_min
    print("position 2: ")
    print(pos)
    return pos


# dxl_goal_position = cal_position()         # Goal position


# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

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

# Enable Dynamixel1 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")

# Enable Dynamixel#2 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL2_ID)

# initialize Dynamixel#1 position
dxl_goal_position = 0
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_GOAL_POSITION, dxl_goal_position)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# initialize Dynamixel#2 position
dxl_goal_position = 0
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_GOAL_POSITION, dxl_goal_position)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

time.sleep(3)

while 1:
    current_time = time.time()
    dxl_goal_position1 = cal_position1(current_time)
    dxl_goal_position1 = int(dxl_goal_position1)
    dxl_goal_position2 = cal_position2(current_time)
    dxl_goal_position2 = int(dxl_goal_position2)

    # Write Dynamixel#1 goal position
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_GOAL_POSITION, dxl_goal_position1)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Write Dynamixel#2 goal position
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_GOAL_POSITION, dxl_goal_position2)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
'''
    while 1:
        # Read present position
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

        print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL_ID, dxl_goal_position, dxl_present_position))

        if not abs(dxl_goal_position - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
            break
'''

# Disable Dynamixel#1 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Disable Dynamixel#2 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Close port
portHandler.closePort()
