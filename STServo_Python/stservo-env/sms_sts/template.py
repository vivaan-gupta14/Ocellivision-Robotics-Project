#--------------------------------------GENERAL IMPORTS----------------------------------------------------------

import sys
import os
import time
print("General Imports Complete")

#---------------------------------------SETTINGS BASED ON OS---------------------------------------------------

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
print("OS Based Settings Complete")
#------------------------------------SERVO PYTHON LIBRARY IMPORT--------------------------------------------

sys.path.append("..")
from scservo_sdk import *                      # Uses SC Servo SDK library
print("SC Servo Library Imported")

#-------------------------------------INITIALIZE SERVO SETTINGS-------------------------------------------------

# Default setting
SCS_ID                          = 1                                 # SC Servo ID :
BAUDRATE                        = 1000000                           # SC Servo default baudrate : 1000000 (1 Mbps)
DEVICENAME                      = '/dev/tty.usbmodem5AAF2197651'    # Check which port is being used on your controller
                                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
                                                                
SCS_MINIMUM_STEP_VALUE          = -4096                             # 4096 steps = 360 degrees. Minimum value is -32766 (~ 8 revolutions)
SCS_MAXIMUM_STEP_VALUE          = 4096                              # 4096 steps = 360 degrees. Minimum value is -32766 (~ 8 revolutions)
                                                                    #Set both values to 0 for unrestricted rotation

SCS_MAXIMUM_TEMPERATURE         = 55                                # SC Servo maximum temperature (55 degrees Celsius)

SCS_MAXIMUM_INPUT_VOLTAGE       = 130                               #13.0 Volts
SCS_MINIMUM_INPUT_VOLTAGE       = 90                                #9.0 Volts

# SCS_PROTECTION_CURRENT          = 2000/6.5                          #2.0 Amps (2000mA)
# SCS_OVERLOAD_CURRENT            = 1700                              #1.7 Amps (1700mA)
# SCS_PROTECTION_TIME             = 10                                #100ms
# SCS_OVERCURRENT_PROTECTION_TIME = 10                                #100ms

# SCS_MINIMUM_STARTING_FORCE      = 120                               #12.0% of max force
# SCS_MAXIMUM_TORQUE              = 300                               #30.0% of max force
# SCS_TORQUE_LIMIT                = 250                               #25.0% of max force
# SCS_PROTECTION_TORQUE           = 20                                #20.0% of max force
# SCS_OVERLOAD_TORQUE             = 15                                #15.0% of max force
# SCS_TORQUE_SWITCH               = 1                                 

SCS_UNLOADING_CONDITIONS        = 63                                # 0b111111: bit0 Voltage, bit1 Sensor, bit2 Temperature,
                                                                     # bit3 Current, bit4 Angle, bit5 Overload -- all 6 enabled
SCS_LED_ALARM_CONDITIONS        = 63                                # Same bit layout as above; LED flashes for all 6 conditions

SCS_MOVING_SPEED                = 1800                              # SC Servo moving speed (1800 steps/sec)
SCS_MOVING_ACC                  = 20                                # SC Servo moving acc (20 steps/sec^2)

SCS_CW_INSENSITIVE_ZONE         = 1
SCS_CCW_INSENSITIVE_ZONE        = 1


print(f"SC Servo ID: {SCS_ID}")
print(f"SC Servo DEVICENAME: {DEVICENAME}")
print(f"SC Servo Minimum Step Value: {SCS_MINIMUM_STEP_VALUE}")
print(f"SC Servo Maximum Step Value: {SCS_MAXIMUM_STEP_VALUE}")
print(f"SC Servo Maximum Temperature: {SCS_MAXIMUM_TEMPERATURE}")
print(f"SC Servo Maximum Input Voltage: {SCS_MAXIMUM_INPUT_VOLTAGE}")
print(f"SC Servo Moving Speed: {SCS_MOVING_SPEED}")
print(f"SC Servo Moving Acc: {SCS_MOVING_ACC}")
print("Default Servo Settings Initialized")

#--------------------------------PORT AND PACKET HANDLER INITIALIZATION-------------------------------------

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

#--------------------------------------APPLY SERVO SETTINGS-----------------------------------------------------

# EEPROM Addresses not already defined in scservo_sdk/sms_sts.py's control table
SMS_STS_MAX_TEMPERATURE_LIMIT   = 13
SMS_STS_MAX_VOLTAGE_LIMIT       = 14
SMS_STS_MIN_VOLTAGE_LIMIT       = 15
SMS_STS_UNLOADING_CONDITION     = 19
SMS_STS_LED_ALARM_CONDITION     = 20

def writeSetting(address, value, num_bytes, name):
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

# EEPROM must be unlocked before these addresses can be written
packetHandler.unLockEprom(SCS_ID)

writeSetting(SMS_STS_MIN_ANGLE_LIMIT_L, packetHandler.scs_toscs(SCS_MINIMUM_STEP_VALUE, 15), 2, "Min Angle Limit")
writeSetting(SMS_STS_MAX_ANGLE_LIMIT_L, packetHandler.scs_toscs(SCS_MAXIMUM_STEP_VALUE, 15), 2, "Max Angle Limit")
writeSetting(SMS_STS_MAX_TEMPERATURE_LIMIT, SCS_MAXIMUM_TEMPERATURE, 1, "Max Temperature Limit")
writeSetting(SMS_STS_MAX_VOLTAGE_LIMIT, SCS_MAXIMUM_INPUT_VOLTAGE, 1, "Max Voltage Limit")
writeSetting(SMS_STS_MIN_VOLTAGE_LIMIT, SCS_MINIMUM_INPUT_VOLTAGE, 1, "Min Voltage Limit")
writeSetting(SMS_STS_UNLOADING_CONDITION, SCS_UNLOADING_CONDITIONS, 1, "Unloading Condition")
writeSetting(SMS_STS_LED_ALARM_CONDITION, SCS_LED_ALARM_CONDITIONS, 1, "LED Alarm Condition")
writeSetting(SMS_STS_CW_DEAD, SCS_CW_INSENSITIVE_ZONE, 1, "CW Insensitive Zone")
writeSetting(SMS_STS_CCW_DEAD, SCS_CCW_INSENSITIVE_ZONE, 1, "CCW Insensitive Zone")

# Re-lock EEPROM to protect the settings just written
packetHandler.LockEprom(SCS_ID)

print("Servo Settings Applied")

#--------------------------------------YOUR CODE HERE-----------------------------------------------------------

try:
    print("Starting the script...")
    pass

#--------------------------------------ENDING THE SCRIPT-------------------------------------------------------

finally:
    # Close port
    portHandler.closePort()
    print("Port Closed")