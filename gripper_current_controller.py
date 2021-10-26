import os
import zmq
import numpy as np
import time
if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect('tcp://127.0.0.1:5560')

################################################################################################################
#setup for the motor
ADDR_MX_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION      = 116
ADDR_MX_PRESENT_POSITION   = 132
ADDR_MX_PRESENT_CURRENT = 126

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = 1                 # Dynamixel ID : 1
# DXL_ID                      = 2                 # Dynamixel ID : 1
BAUDRATE                    = 57600          # Dynamixel default baudrate : 57600
# DEVICENAME                  = '/dev/tty.usbserial-FT2N061F'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex)

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
# DXL_MINIMUM_POSITION_VALUE  = 2150           # Dynamixel will rotate between this value
# DXL_MAXIMUM_POSITION_VALUE  = 3250            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#### For the XM430 ON STRETCH ROBOT ##########
# DXL_MINIMUM_POSITION_VALUE  = 1550           # Dynamixel will rotate between this value
# DXL_MAXIMUM_POSITION_VALUE  = 2250            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#### For the XM430 ON STRETCH ROBOT ##########
DXL_MINIMUM_POSITION_VALUE  = 1500           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 2850            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 5                # Dynamixel moving status threshold

index = 0
dxl_goal_position_range_ = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position

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
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

# Enable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")


# Changing operating mode
ADDR_OPERATING_MODE= 11
OP_MODE_POSITION= 5
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, OP_MODE_POSITION)

# set the current limit
ADDR_CURRENT_LIMIT = 38
CURRENT_LIMIT_UPBOUND = 1193
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_CURRENT_LIMIT, CURRENT_LIMIT_UPBOUND)

#SET THE VELOCITU LIMIT
ADDR_VELOCITY_LIMIT = 44
VELOCITY_LIMIT_UPBOUND = 1023
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_VELOCITY_LIMIT, VELOCITY_LIMIT_UPBOUND)

#SET THE MAX POSITION LIMIT
ADDR_MAX_POSITION_LIMIT = 48
MAX_POSITION_LIMIT_UPBOUND = DXL_MAXIMUM_POSITION_VALUE
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_MAX_POSITION_LIMIT, MAX_POSITION_LIMIT_UPBOUND)

#SET THE MIN POSITION LIMIT
ADDR_MIN_POSITION_LIMIT = 52
MIN_POSITION_LIMIT_UPBOUND = DXL_MINIMUM_POSITION_VALUE
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_MIN_POSITION_LIMIT, MIN_POSITION_LIMIT_UPBOUND)

ADDR_GOAL_CURRENT = 102

#GOAL_CURRENT_MINPOSITION = 1

# SET THE GOAL VELOCITY
ADDR_GOAL_VELOCITY = 104
GOAL_VELOCITY_MAXPOSITION = 1023
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_VELOCITY, GOAL_VELOCITY_MAXPOSITION)


ADDR_ACCELERATION_PROFILE = 108
ACCELERATION_ADDRESS_POSITION= 0
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_ACCELERATION_PROFILE, ACCELERATION_ADDRESS_POSITION)

ADDR_VELOCITY_PROFILE = 112
VELOCITY_ADDRESS_POSITION= 0
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_VELOCITY_PROFILE, VELOCITY_ADDRESS_POSITION)

# Enable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel torque control has been successfully connected")

# dxl_goal_position = 2200
dxl_current_torque = 200
max_dxl_current_torque = 400
pos_increasment = 50
sign = 0
dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION)
dxl_present_current, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_CURRENT)
print("present position is : !!!!!!!!!!!!!!!!!!!!!!!!!", dxl_present_position)
dxl_goal_position = DXL_MINIMUM_POSITION_VALUE + 50
dxl_goal_position_temp = dxl_goal_position
print(dxl_goal_position, dxl_present_position)

print_count = 0
while 1:
    # print("Press any key to continue! (or press ESC to quit!)")
    start = time.time()
    # if getch() == chr(0x1b):
    #    break

    # Write goal position
    assert(dxl_goal_position <= DXL_MAXIMUM_POSITION_VALUE and dxl_goal_position >= DXL_MINIMUM_POSITION_VALUE), "Goal position {} is not in range !!!!!".format(dxl_goal_position)
    # dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_CURRENT, dxl_current_torque)
    # dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position)
    # if dxl_comm_result != COMM_SUCCESS:
    #     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    # elif dxl_error != 0:
    #     print("%s" % packetHandler.getRxPacketError(dxl_error))
    #
    # while 1:
    #     # Read present position
    #     dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION)
    #     dxl_present_current, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_CURRENT)
    #     if dxl_comm_result != COMM_SUCCESS:
    #         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    #     elif dxl_error != 0:
    #         print("%s" % packetHandler.getRxPacketError(dxl_error))
    #
    #     print("[ID:%03d] GoalPos:%03d  PresPos:%03d ---- GoalCurr:%d  PresCurr:%d" % (DXL_ID, dxl_goal_position, dxl_present_position, dxl_current_torque, dxl_present_current))
    #
    #     if not abs(dxl_goal_position - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
    #         break

    if(abs(packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION)[0] - dxl_goal_position) >= DXL_MOVING_STATUS_THRESHOLD):
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION)
        dxl_goal_position_temp = dxl_present_position
        if (dxl_present_position <= dxl_goal_position):
            dxl_goal_position_temp += min(pos_increasment, dxl_goal_position - dxl_present_position)
        else:
            dxl_goal_position_temp -= min(pos_increasment, dxl_present_position - dxl_goal_position)
        assert(dxl_goal_position_temp <= DXL_MAXIMUM_POSITION_VALUE and dxl_goal_position >= DXL_MINIMUM_POSITION_VALUE), "Goal position temp is not in range !!!!!"
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_CURRENT, dxl_current_torque)
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position_temp)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

        # while 1:
        #     # Read present position
        # dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION)
        # dxl_present_current, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_CURRENT)
        #     if dxl_present_current & (1 << (16-1)):
        #         dxl_present_current -= 1 << 16
        #     if dxl_comm_result != COMM_SUCCESS:
        #         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        #     elif dxl_error != 0:
        #         print("%s" % packetHandler.getRxPacketError(dxl_error))
        #
        #     print(hex(dxl_present_current))
        #     print(type(hex(dxl_present_current)))
        # print("first brach: [ID:%03d] GoalPos:%03d  GoalPosTemp:%03d  PresPos:%03d ---- GoalCurr:%d  PresCurr:%f" % (DXL_ID, dxl_goal_position, dxl_goal_position_temp, dxl_present_position, dxl_current_torque, dxl_present_current))
        #
        #     if not abs(dxl_goal_position_temp - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
        #         break

    # while 1:
    #     # Read present position
    #     dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION)
    #     dxl_present_current, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_CURRENT)
    #     if dxl_present_current & (1 << (16-1)):
    #         dxl_present_current -= 1 << 16
    #     if dxl_comm_result != COMM_SUCCESS:
    #         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    #     elif dxl_error != 0:
    #         print("%s" % packetHandler.getRxPacketError(dxl_error))
    #     print(hex(dxl_present_current))
    #     print(type(hex(dxl_present_current)))
    #     print("[ID:%03d] GoalPos:%03d GoalPosTemp:%03d PresPos:%03d ---- GoalCurr:%d  PresCurr:%d" % (DXL_ID, dxl_goal_position, dxl_goal_position_temp, dxl_present_position, dxl_current_torque, dxl_present_current))
    #
    #     if not abs(dxl_goal_position - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
    #         break
    else:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position)
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_CURRENT, dxl_current_torque)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Change goal current
    data = np.array(dxl_current_torque)
    md = dict(dtype = str(data.dtype))
    socket.send_json(md, 0|zmq.SNDMORE)
    socket.send(data, 0|zmq.SNDMORE, copy=True, track=False)
    # Change goal position
    data = np.array(dxl_present_position)
    md = dict(
        dtype = str(data.dtype)
    )
    socket.send_json(md, 0|zmq.SNDMORE)
    socket.send(data, 0, copy=True, track=False)
    msg = socket.recv(flags=0, copy=True, track=False)
    buf = memoryview(msg)
    buf_array = np.frombuffer(buf, dtype='int')
    dxl_goal_position = buf_array[0]
    dxl_current_torque = buf_array[1]
    dxl_current_torque = min(dxl_current_torque, 400)
    # if index == 0:
    #     index = 1
    # else:
    #     index = 0

    end = time.time()
    print(end-start)
    print_count += 1
    if print_count >= 20:
         # = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION)
        results = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION)
        # if len(results) == 3:
        #     dxl_present_position, dxl_comm_result, dxl_error = results
        # else:
        #     print(results)
        print(results)
        dxl_present_current, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_MX_PRESENT_CURRENT)
        print("second branch: [ID:%03d] GoalPos:%03d GoalPosTemp:%03d  ---- GoalCurr:%d  PresCurr:%d" % (DXL_ID, dxl_goal_position, dxl_goal_position_temp,  dxl_current_torque, dxl_present_current))
        print(dxl_goal_position, "          ", dxl_current_torque)

# Disable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE , TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Close port
portHandler.closePort()
