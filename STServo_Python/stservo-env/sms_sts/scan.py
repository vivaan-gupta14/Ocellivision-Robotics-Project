#!/usr/bin/env python
#
# *********     Scan Example      *********
#
# Pings every possible servo ID and reports which ones respond.
# Use this when you don't know a servo's SCS_ID.
#
# Available SC Servo model on this example : All models using Protocol SC

import sys
import os

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

sys.path.append("..")
from scservo_sdk import *                   # Uses SC Servo SDK library

# Default setting
BAUDRATE                = 1000000             # SC Servo default baudrate : 1000000
DEVICENAME              = '/dev/tty.usbmodem5AAF2197651'    # Check which port is being used on your controller
                                            # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Get methods and members of Protocol
packetHandler = sms_sts(portHandler)

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

# Scan every possible ID (0-252, excluding the 253/254 broadcast/reserved IDs)
print("Scanning IDs 0-252 ...")
found_ids = []
for scs_id in range(0, 253):
    scs_model_number, scs_comm_result, scs_error = packetHandler.ping(scs_id)
    if scs_comm_result == COMM_SUCCESS:
        print("[ID:%03d] ping Succeeded. SC Servo model number : %d" % (scs_id, scs_model_number))
        found_ids.append(scs_id)

if found_ids:
    print("\nFound servo ID(s): %s" % found_ids)
else:
    print("\nNo servos found. Check wiring, baudrate, and DEVICENAME.")

# Close port
portHandler.closePort()
