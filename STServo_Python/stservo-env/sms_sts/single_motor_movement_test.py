
#-----------------------------------------------------GENERAL IMPORTS----------------------------------------------------------

import sys
import os
import time
import random
print("General Imports Complete\n")

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


#-------------------------------------------------------SERVO PYTHON LIBRARY IMPORT--------------------------------------------

sys.path.append("..")
from scservo_sdk import *                      # Uses SC Servo SDK library
print("SC Servo Library Imported\n")

#---------------------------------------------------GET AND SET SETTINGS FUNCTIONS-----------------------------------------------------------

def setSetting(address, value, num_bytes, name):
    if num_bytes == 1:
        scs_comm_result, scs_error = packetHandler.write1ByteTxRx(SCS_ID, address, value)
    else:
        scs_comm_result, scs_error = packetHandler.write2ByteTxRx(SCS_ID, address, value)

    if scs_comm_result != COMM_SUCCESS:
        print(f"[{name}] {packetHandler.getTxRxResult(scs_comm_result)}")
    elif scs_error != 0:
        print(f"[{name}] {packetHandler.getRxPacketError(scs_error)}")
    else:
        print(f"[{name}] Set to {value}")

def getSetting(address, num_bytes, name):
    if num_bytes == 1:
        value, scs_comm_result, scs_error = packetHandler.read1ByteTxRx(SCS_ID, address)
    else:
        value, scs_comm_result, scs_error = packetHandler.read2ByteTxRx(SCS_ID, address)

    if scs_comm_result != COMM_SUCCESS:
        print(f"[{name}] {packetHandler.getTxRxResult(scs_comm_result)}")
    elif scs_error != 0:
        print(f"[{name}] {packetHandler.getRxPacketError(scs_error)}")
    else:
        print(f"[{name}] Read {value}")
    return value
#-----------------------------------------------------MOVE TO POSITION FUNCTION-----------------------------------------------------------

def move_to_position(packetHandler, scs_id, goal_position, moving_speed, moving_acc):
    # Write SC Servo goal position/moving speed/moving acc
    scs_comm_result, scs_error = packetHandler.WritePosEx(scs_id, goal_position, moving_speed, moving_acc)
    if scs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(scs_comm_result))
    elif scs_error != 0:
        print("%s" % packetHandler.getRxPacketError(scs_error))

    has_moved = False
    while 1:
        # Read SC Servo present position
        scs_present_position, scs_present_speed, scs_comm_result, scs_error = packetHandler.ReadPosSpeed(scs_id)
        if scs_comm_result != COMM_SUCCESS:
            print(packetHandler.getTxRxResult(scs_comm_result))
        else:
            print("[ID:%03d] GoalPos:%d PresPos:%d PresSpd:%d" % (scs_id, goal_position, scs_present_position, scs_present_speed))
        if scs_error != 0:
            print(packetHandler.getRxPacketError(scs_error))

        # Read SC Servo moving status
        moving, scs_comm_result, scs_error = packetHandler.ReadMoving(scs_id)
        if scs_comm_result != COMM_SUCCESS:
            print(packetHandler.getTxRxResult(scs_comm_result))

        if moving == 1:
            has_moved = True

        # Trust moving==0 as "arrived" once we've seen it move, OR if we're already
        # within the servo's dead zone (it may never set moving==1 if the goal is
        # close enough that it decides no move is needed).
        if moving == 0 and (has_moved or abs(scs_present_position - goal_position) <= SCS_POSITION_TOLERANCE):
            break

#-------------------------------------------------------INITIALIZE SERVO SETTINGS-------------------------------------------------

# Default setting
SCS_ID                          = 10                                # SC Servo ID : 10
BAUDRATE                        = 1000000                           # SC Servo default baudrate : 1000000 (1 Mbps)
DEVICENAME                      = '/dev/tty.usbmodem5AAF2197651'    # Check which port is being used on your controller
                                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
SCS_MOVING_SPEED                = 1200                              # SC Servo moving speed (2400 steps/sec)
SCS_MOVING_ACC                  = 10                                # SC Servo moving acc (50 steps/sec^2)
SCS_POSITION_TOLERANCE          = 3                                 # Dead-zone: positions within this range count as "arrived"

DESIRED_CENTER_RAW              = 3860                              # The raw position that should become the new center
RANGE                           = 800                               # The maximum offset from the center
CORRECTION                      = DESIRED_CENTER_RAW - 2048         # The position correction to be applied
SCS_CALIBRATED_CENTER           = 2048                              # The position value that becomes the "center" position after correction
SCS_MIN_POSITION                = SCS_CALIBRATED_CENTER - RANGE     # The minimum position value after correction
SCS_MAX_POSITION                = SCS_CALIBRATED_CENTER + RANGE     # The maximum position value after correction

print(f"SC Servo ID: {SCS_ID}")
print(f"SC Servo BAUDRATE: {BAUDRATE}")
print(f"SC Servo DEVICENAME: {DEVICENAME}")
print(f"SC Servo Moving Speed: {SCS_MOVING_SPEED}")
print(f"SC Servo Moving Acc: {SCS_MOVING_ACC}")
print(f"SC Servo Position Tolerance: {SCS_POSITION_TOLERANCE}")
print(f"SC Servo Desired Center (Raw): {DESIRED_CENTER_RAW}")
print(f"SC Servo Range From Center: {RANGE}")
print(f"SC Servo Correction: {CORRECTION}")
print(f"SC Servo Calibrated Center: {SCS_CALIBRATED_CENTER}")
print(f"SC Servo Minimum Position: {SCS_MIN_POSITION}")
print(f"SC Servo Maximum Position: {SCS_MAX_POSITION}")
print("")


#--------------------------------------------------PORT AND PACKET HANDLER INITIALIZATION-------------------------------------

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)
print("Port Handler Initialized")

# Initialize PacketHandler instance
# Get methods and members of Protocol
#Converts code instructions into bytes
packetHandler = sms_sts(portHandler)
print("Packet Handler Initialized")

time.sleep(5)

# Open port with error checking
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate with error checking
if portHandler.setBaudRate(BAUDRATE):
    print(f"Succeeded to change the baudrate to {BAUDRATE}")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

print("")

#------------------------------------------------------------POSITION CORRECTION----------------------------------------------

# Move to the desired physical orientation first
print(f"Moving to target physical center angle ({DESIRED_CENTER_RAW})...")
move_to_position(packetHandler, SCS_ID, DESIRED_CENTER_RAW, SCS_MOVING_SPEED, SCS_MOVING_ACC)
time.sleep(2.0) # Ensure it has completely stopped vibrating

print("--- Starting Offset Calibration Process ---")

# Step 1: Unlock EEPROM Configuration Memory (Address 55 = 0)
setSetting(address=55, value=0, num_bytes=1, name="EEPROM Lock")
time.sleep(0.1)

# Step 2: Write the positive offset delta (1702) to Register 31 (2 Bytes)
setSetting(address=31, value=CORRECTION, num_bytes=2, name="Position Offset (OFS)")
time.sleep(0.1)

# Step 3: Lock EEPROM back down (Address 55 = 1) to protect the memory
setSetting(address=55, value=1, num_bytes=1, name="EEPROM Lock")
time.sleep(0.2)

# Step 4: Verify the offset was written successfully
getSetting(address=31, num_bytes=2, name="Verified Offset")

# Verify that the servo now reads its current resting position as 2048
scs_present_position, _, _, _ = packetHandler.ReadPosSpeed(SCS_ID)
print(f"Current physical position reading after calibration reboot: {scs_present_position}")

# Now this call will execute smoothly. The servo is already physically there,
# and its internal register accurately confirms it is sitting at 2048.
print("Confirming alignment with center target...")
move_to_position(packetHandler, SCS_ID, SCS_CALIBRATED_CENTER, SCS_MOVING_SPEED, SCS_MOVING_ACC)



print(f"--- Calibration Complete --- \n")

time.sleep(5)



#------------------------------------------------------------MAIN CONTROL LOOP------------------------------------------------

try:
    # Demo sweep now stays within the calibrated safe range - after the OFS
    # correction above, raw values like 0/1024/4095 no longer correspond to the
    # old full-rotation sweep and would drive the arm straight into its
    # mechanical stops.
    print(f"Moving to low limit ({SCS_MIN_POSITION})")
    move_to_position(packetHandler, SCS_ID, SCS_MIN_POSITION, SCS_MOVING_SPEED, SCS_MOVING_ACC)
    time.sleep(2)

    print(f"Moving to center position ({SCS_CALIBRATED_CENTER})")
    move_to_position(packetHandler, SCS_ID, SCS_CALIBRATED_CENTER, SCS_MOVING_SPEED, SCS_MOVING_ACC)
    time.sleep(2)

    print(f"Moving to high limit ({SCS_MAX_POSITION})")
    move_to_position(packetHandler, SCS_ID, SCS_MAX_POSITION, SCS_MOVING_SPEED, SCS_MOVING_ACC)
    time.sleep(2)

    print(f"Moving to center position ({SCS_CALIBRATED_CENTER})")
    move_to_position(packetHandler, SCS_ID, SCS_CALIBRATED_CENTER, SCS_MOVING_SPEED, SCS_MOVING_ACC)
    time.sleep(2)

    while 1:
        userInput = input(f"Enter target position (from {-RANGE} to {RANGE}), or 'q' to quit: ")
        if userInput.strip().lower() == 'q':
            break

        try:
            userIntInput = int(userInput)
        except ValueError:
            print(f"'{userIntInput}' is not a valid number. Please try again.\n")
            continue
        
        goal_position = userIntInput + 2048

        if goal_position < SCS_MIN_POSITION or goal_position > SCS_MAX_POSITION:
            print(f"Position {userIntInput} is out of range. Must be between {-RANGE} and {RANGE}.\n")
            continue

        move_to_position(packetHandler, SCS_ID, goal_position, SCS_MOVING_SPEED, SCS_MOVING_ACC)

#-----------------------------------------------------ENDING THE SCRIPT-------------------------------------------------------

finally:
    # Close port
    portHandler.closePort()
    print("Port Closed")