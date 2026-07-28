
#-----------------------------------------------------GENERAL IMPORTS----------------------------------------------------------

import sys
import os
import time
import random
print("General Imports Complete\n")
time.sleep(2)

#-------------------------------------------------------SETTINGS BASED ON OS---------------------------------------------------

if os.name == 'nt': #'nt' means Windows
    print("OS detected as Windows")
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
        
else:
    print("OS detected as Linux/Mac")
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
print("OS Based Settings Complete\n")
time.sleep(2)

#-------------------------------------------------------SERVO PYTHON LIBRARY IMPORT--------------------------------------------

sys.path.append("..")
from scservo_sdk import *                      # Uses SC Servo SDK library
print("SC Servo Library Imported\n")
time.sleep(2)

#-------------------------------------------------------INITIALIZE SERVO SETTINGS-------------------------------------------------

# Default setting
SCS_ID                          = 10                                # SC Servo ID : 10
BAUDRATE                        = 1000000                           # SC Servo default baudrate : 1000000 (1 Mbps)
DEVICENAME                      = '/dev/tty.usbmodem5AAF2197651'    # Check which port is being used on your controller
                                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
SCS_MINIMUM_POSITION_VALUE  = 0             # Min SC Servo rotation value (0 degrees)
SCS_MAXIMUM_POSITION_VALUE  = 4095          # Max SC Servo rotation value (360 degrees)
SCS_MOVING_SPEED            = 2400          # SC Servo moving speed (2400 steps/sec)
SCS_MOVING_ACC              = 50            # SC Servo moving acc (50 steps/sec^2)

print(f"SC Servo ID: {SCS_ID}")
print(f"SC Servo DEVICENAME: {DEVICENAME}")
print(f"SC Servo BAUDRATE: {BAUDRATE}")
print(f"SC Servo Minimum Position Value: {SCS_MINIMUM_POSITION_VALUE}")
print(f"SC Servo Maximum Position Value: {SCS_MAXIMUM_POSITION_VALUE}")
print(f"SC Servo Moving Speed: {SCS_MOVING_SPEED}")
print(f"SC Servo Moving Acc: {SCS_MOVING_ACC}")
print("")
time.sleep(5)

#--------------------------------------------------PORT AND PACKET HANDLER INITIALIZATION-------------------------------------

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)
print("Port Handler Initialized")
time.sleep(1)

# Initialize PacketHandler instance
# Get methods and members of Protocol
#Converts code instructions into bytes
packetHandler = sms_sts(portHandler)
print("Packet Handler Initialized")
time.sleep(1)

# Open port with error checking
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

time.sleep(1)

# Set port baudrate with error checking
if portHandler.setBaudRate(BAUDRATE):
    print(f"Succeeded to change the baudrate to {BAUDRATE}")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

print("")
time.sleep(5)

#-----------------------------------------------------YOUR CODE HERE-----------------------------------------------------------

try:
    print("Starting the script...")
    while 1:
        user_input = input(f"Enter target position (from {SCS_MINIMUM_POSITION_VALUE} to {SCS_MAXIMUM_POSITION_VALUE}), or 'q' to quit: ")
        if user_input.strip().lower() == 'q':
            break

        try:
            goal_position = int(user_input)
        except ValueError:
            print(f"'{user_input}' is not a valid number. Please try again.\n")
            continue

        if goal_position < SCS_MINIMUM_POSITION_VALUE or goal_position > SCS_MAXIMUM_POSITION_VALUE:
            print(f"Position {goal_position} is out of range. Must be between {SCS_MINIMUM_POSITION_VALUE} and {SCS_MAXIMUM_POSITION_VALUE}.\n")
            continue

        # Write SC Servo goal position/moving speed/moving acc
        scs_comm_result, scs_error = packetHandler.WritePosEx(SCS_ID, goal_position, SCS_MOVING_SPEED, SCS_MOVING_ACC)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % packetHandler.getRxPacketError(scs_error))

        while 1:
            # Read SC Servo present position
            scs_present_position, scs_present_speed, scs_comm_result, scs_error = packetHandler.ReadPosSpeed(SCS_ID)
            if scs_comm_result != COMM_SUCCESS:
                print(packetHandler.getTxRxResult(scs_comm_result))
            else:
                print("[ID:%03d] GoalPos:%d PresPos:%d PresSpd:%d" % (SCS_ID, goal_position, scs_present_position, scs_present_speed))
            if scs_error != 0:
                print(packetHandler.getRxPacketError(scs_error))

            # Read SC Servo moving status
            moving, scs_comm_result, scs_error = packetHandler.ReadMoving(SCS_ID)
            if scs_comm_result != COMM_SUCCESS:
                print(packetHandler.getTxRxResult(scs_comm_result))

            if moving==0:
                break

#-----------------------------------------------------ENDING THE SCRIPT-------------------------------------------------------

finally:
    # Close port
    portHandler.closePort()
    print("Port Closed")