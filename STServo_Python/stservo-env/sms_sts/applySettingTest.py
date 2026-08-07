
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

def move_to_position(packetHandler, scs_id, goal_position, moving_speed, moving_acc, tolerance):
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
        if moving == 0 and (has_moved or abs(scs_present_position - goal_position) <= tolerance):
            break

#-------------------------------------------------------INITIALIZE SERVO SETTINGS-------------------------------------------------

# Default setting
SCS_ID                          = 6                                 # SC Servo ID : 10
BAUDRATE                        = 1000000                           # SC Servo default baudrate : 1000000 (1 Mbps)
DEVICENAME                      = '/dev/tty.usbmodem5AAF2197651'    # Check which port is being used on your controller
                                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"


print(f"SC Servo ID: {SCS_ID}")
print(f"SC Servo BAUDRATE: {BAUDRATE}")
print(f"SC Servo DEVICENAME: {DEVICENAME}")
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
time.sleep(10)


#---------------------------------------------------------SERVO SETTINGS------------------------------------------------------


# EEPROM must be unlocked before these addresses can be written
packetHandler.unLockEprom(SCS_ID)

#----------------------------------Apply Settings

setSetting(address=40, value = 1, num_bytes= 1, name = "Torque Switch")
setSetting(address=33, value = 0, num_bytes= 1, name = "Operation Mode")
setSetting(address=30, value = 5, num_bytes = 1, name = "Angle Resolution")
# 348/3748 is a bounded (non-multi-turn) range, but at Angle Resolution=5 that's
# (3748-348)*5 = 17000 raw steps - about 4 revolutions - which covers the lead
# screw's several-revolution physical travel without needing 0/0 multi-turn mode.
setSetting(address=9, value = 348, num_bytes=2, name = "Min Angle Limit")
setSetting(address=11, value = 3748, num_bytes=2, name = "Max Angle Limit")

# Multi-turn also needs Phase (0x12/18) BIT4 set so Present Position feedback reflects
# the larger angle instead of wrapping every revolution. Read-modify-write so we don't
# clobber whatever other bits (e.g. direction) are already set in this byte.
_phase = getSetting(address=18, num_bytes=1, name = "Phase (before)")
setSetting(address=18, value = _phase | 0x10, num_bytes=1, name = "Phase (multi-turn bit4 set)")

#-----------------------------------------------

# Re-lock EEPROM to protect the settings just written
packetHandler.LockEprom(SCS_ID)

#----------------------------------Verify Settings

getSetting(address=30, num_bytes=1, name = "Angle Resolution")
getSetting(address=33, num_bytes=1, name = "Operation Mode")
getSetting(address=40, num_bytes= 1, name = "Torque Switch")
getSetting(address=9, num_bytes=2, name = "Min Angle Limit")
getSetting(address=11, num_bytes=2, name = "Max Angle Limit")
getSetting(address=18, num_bytes=1, name = "Phase")

#------------------------------------------------
# Deliberately NOT re-running Torque Switch=128 ("correct current position to 2048")
# here: that command only pins the RAW position, ignoring Angle Resolution, so at
# resolution 5 it would leave Present Position at ~409, not 2048 - it was actually the
# source of that exact bug when this script used to call it. Calibration (including the
# Angle-Resolution-aware Position Correction offset) is expected to already have been
# done by servoRangeAndCenterTest.py; the safety check below verifies that rather than
# blindly re-anchoring.


try:
    # Safety check: only proceed with the move sequence if the servo is
    # currently sitting at the known-safe start position (2048). Anything
    # else means we don't trust where the arm actually is, so abort instead
    # of commanding a move from an unknown position.
    scs_present_position, scs_present_speed, scs_comm_result, scs_error = packetHandler.ReadPosSpeed(SCS_ID)
    print(f"Present Position: {scs_present_position}")
    time.sleep(1)
    if scs_comm_result != COMM_SUCCESS:
        raise RuntimeError(f"Safety check failed: could not read present position ({packetHandler.getTxRxResult(scs_comm_result)})")
    if scs_error != 0:
        raise RuntimeError(f"Safety check failed: servo error reading present position ({packetHandler.getRxPacketError(scs_error)})")
    if scs_present_position != 2048:
        raise RuntimeError(f"Safety check failed: present position {scs_present_position} != 2048, refusing to move")
    print(f"[Safety Check] Present position confirmed at {scs_present_position}")

    time.sleep(5)

    #------------------------------------------------------------MAIN CONTROL LOOP------------------------------------------------

    print("moving to center position (2048)")
    move_to_position(packetHandler, SCS_ID, 2048, 200, 10, 1)
    time.sleep(2)
    print("moving to max position (3648)")
    move_to_position(packetHandler, SCS_ID, 3648, 200, 10, 1)
    time.sleep(2)
    print("moving to center position (2048)")
    move_to_position(packetHandler, SCS_ID, 2048, 200, 10, 1)
#-----------------------------------------------------ENDING THE SCRIPT-------------------------------------------------------

finally:
    # Close port
    portHandler.closePort()
    print("Port Closed")