#--------------------------------------GENERAL IMPORTS----------------------------------------------------------

import sys
import os
print("General Imports Complete")

#------------------------------------SERVO PYTHON LIBRARY IMPORT--------------------------------------------

sys.path.append("..")
from scservo_sdk import *                      # Uses SC Servo SDK library
print("SC Servo Library Imported")

#-------------------------------------INITIALIZE SERVO SETTINGS-------------------------------------------------

SCS_ID       = 6                                 # SC Servo ID : change to match the motor you need to stop
BAUDRATE     = 1000000                           # SC Servo default baudrate : 1000000 (1 Mbps)
DEVICENAME   = '/dev/tty.usbmodem5AAF2197651'    # Check which port is being used on your controller

#--------------------------------PORT AND PACKET HANDLER INITIALIZATION-------------------------------------

portHandler = PortHandler(DEVICENAME)
packetHandler = sms_sts(portHandler)

if not portHandler.openPort():
    print("Failed to open the port")
    quit()

if not portHandler.setBaudRate(BAUDRATE):
    print("Failed to change the baudrate")
    quit()

#--------------------------------------STOP THE MOTOR------------------------------------------------------------

# Cutting torque (rather than commanding a new goal position) is the safest immediate
# stop: it works even if the servo's last goal position was bad, and doesn't depend on
# knowing where the servo currently is.
scs_comm_result, scs_error = packetHandler.write1ByteTxRx(SCS_ID, SMS_STS_TORQUE_ENABLE, 0)
if scs_comm_result != COMM_SUCCESS:
    print(packetHandler.getTxRxResult(scs_comm_result))
elif scs_error != 0:
    print(packetHandler.getRxPacketError(scs_error))
else:
    print(f"[ID:{SCS_ID:03d}] Torque disabled - motor stopped and free to move by hand.")

scs_present_position, scs_comm_result, scs_error = packetHandler.ReadPos(SCS_ID)
if scs_comm_result == COMM_SUCCESS and scs_error == 0:
    print(f"[ID:{SCS_ID:03d}] Present raw position: {scs_present_position}")

portHandler.closePort()
print("Port Closed")
