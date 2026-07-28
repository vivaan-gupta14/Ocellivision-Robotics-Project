
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
                                                                
SCS_MINIMUM_STEP_VALUE          = 0                             # 4096 steps = 360 degrees. Minimum value is -32766 (~ 8 revolutions)
SCS_MAXIMUM_STEP_VALUE          = 4095                              # 4096 steps = 360 degrees. Maximum value is 32766 (~ 8 revolutions)
                                                                    #Set both values to 0 for unrestricted rotation

SCS_MAXIMUM_TEMPERATURE         = 55                                # SC Servo maximum temperature (55 degrees Celsius)

SCS_MAXIMUM_INPUT_VOLTAGE       = 130                               #13.0 Volts
SCS_MINIMUM_INPUT_VOLTAGE       = 90                                #9.0 Volts

# SCS_PROTECTION_CURRENT          = 2000/6.5                        #2.0 Amps (2000mA)
# SCS_OVERLOAD_CURRENT            = 1700                            #1.7 Amps (1700mA)
# SCS_PROTECTION_TIME             = 10                              #100ms
# SCS_OVERCURRENT_PROTECTION_TIME = 10                              #100ms

# SCS_MINIMUM_STARTING_FORCE      = 120                             #12.0% of max force
# SCS_MAXIMUM_TORQUE              = 300                             #30.0% of max force
# SCS_TORQUE_LIMIT                = 250                             #25.0% of max force
# SCS_PROTECTION_TORQUE           = 20                              #20.0% of max force
# SCS_OVERLOAD_TORQUE             = 15                              #15.0% of max force
# SCS_TORQUE_SWITCH               = 1                                 

SCS_UNLOADING_CONDITIONS        = 47                                # 0b111111: bit0 Voltage, bit1 Sensor, bit2 Temperature,
                                                                     # bit3 Current, bit4 Angle, bit5 Overload -- all 6 enabled
SCS_LED_ALARM_CONDITIONS        = 63                                # Same bit layout as above; LED flashes for all 6 conditions

SCS_MOVING_SPEED                = 1800                              # SC Servo moving speed (1800 steps/sec)
SCS_MOVING_ACC                  = 20                                # SC Servo moving acc (20 steps/sec^2)

SCS_CW_INSENSITIVE_ZONE         = 0
SCS_CCW_INSENSITIVE_ZONE        = 0


print(f"SC Servo ID: {SCS_ID}")
print(f"SC Servo DEVICENAME: {DEVICENAME}")
print(f"SC Servo BAUDRATE: {BAUDRATE}")
print(f"SC Servo Minimum Step Value: {SCS_MINIMUM_STEP_VALUE}")
print(f"SC Servo Maximum Step Value: {SCS_MAXIMUM_STEP_VALUE}")
print(f"SC Servo Maximum Temperature: {SCS_MAXIMUM_TEMPERATURE}")
print(f"SC Servo Maximum Input Voltage: {SCS_MAXIMUM_INPUT_VOLTAGE}")
print(f"SC Servo Minimum Input Voltage: {SCS_MINIMUM_INPUT_VOLTAGE}")
print(f"SC Servo Moving Speed: {SCS_MOVING_SPEED}")
print(f"SC Servo Moving Acc: {SCS_MOVING_ACC}")
print(f"SC Servo CW Insensitive Zone: {SCS_CW_INSENSITIVE_ZONE}")
print(f"SC Servo CCW Insensitive Zone: {SCS_CCW_INSENSITIVE_ZONE}")
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

#-----------------------------------------------------APPLY SERVO SETTINGS-----------------------------------------------------

# EEPROM Addresses not already defined in scservo_sdk/sms_sts.py's control table
SMS_STS_MAX_TEMPERATURE_LIMIT   = 13
SMS_STS_MAX_VOLTAGE_LIMIT       = 14
SMS_STS_MIN_VOLTAGE_LIMIT       = 15
SMS_STS_UNLOADING_CONDITION     = 19
SMS_STS_LED_ALARM_CONDITION     = 20

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

# EEPROM must be unlocked before these addresses can be written
packetHandler.unLockEprom(SCS_ID)

convertedMinAngle = packetHandler.scs_toscs(SCS_MINIMUM_STEP_VALUE, 15)
convertedMaxAngle = packetHandler.scs_toscs(SCS_MAXIMUM_STEP_VALUE, 15)

print(f"SCS_MINIMUM_STEP_VALUE: {SCS_MINIMUM_STEP_VALUE}")
print(f"Converted: {convertedMinAngle}")
print(f"Converted back: {packetHandler.scs_tohost(convertedMinAngle, 15)}\n")
time.sleep(3)


print(f"SCS_MAXIMUM_STEP_VALUE: {SCS_MAXIMUM_STEP_VALUE}")
print(f"Converted: {convertedMaxAngle}")
print(f"Converted back: {packetHandler.scs_tohost(convertedMaxAngle, 15)}\n")
time.sleep(3)

setSetting(SMS_STS_MIN_ANGLE_LIMIT_L, convertedMinAngle, 2, "Min Angle Limit")
time.sleep(1)
setSetting(SMS_STS_MAX_ANGLE_LIMIT_L, convertedMaxAngle, 2, "Max Angle Limit")
time.sleep(1)
setSetting(SMS_STS_MAX_TEMPERATURE_LIMIT, SCS_MAXIMUM_TEMPERATURE, 1, "Max Temperature Limit")
time.sleep(1)
setSetting(SMS_STS_MAX_VOLTAGE_LIMIT, SCS_MAXIMUM_INPUT_VOLTAGE, 1, "Max Voltage Limit")
time.sleep(1)
setSetting(SMS_STS_MIN_VOLTAGE_LIMIT, SCS_MINIMUM_INPUT_VOLTAGE, 1, "Min Voltage Limit")
time.sleep(1)
setSetting(SMS_STS_UNLOADING_CONDITION, SCS_UNLOADING_CONDITIONS, 1, "Unloading Condition")
time.sleep(1)
setSetting(SMS_STS_LED_ALARM_CONDITION, SCS_LED_ALARM_CONDITIONS, 1, "LED Alarm Condition")
time.sleep(1)
setSetting(SMS_STS_CW_DEAD, SCS_CW_INSENSITIVE_ZONE, 1, "CW Insensitive Zone")
time.sleep(1)
setSetting(SMS_STS_CCW_DEAD, SCS_CCW_INSENSITIVE_ZONE, 1, "CCW Insensitive Zone")
time.sleep(1)

# Re-lock EEPROM to protect the settings just written
packetHandler.LockEprom(SCS_ID)

print("Servo Settings Applied\n")
time.sleep(5)

#--------------------------------------------SERVO SETTINGS READBACK VERIFICATION-------------------------------------------------------

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

getSetting(SMS_STS_MIN_ANGLE_LIMIT_L, 2, "Min Angle Limit")
time.sleep(1)
getSetting(SMS_STS_MAX_ANGLE_LIMIT_L, 2, "Max Angle Limit")
time.sleep(1)
getSetting(SMS_STS_MAX_TEMPERATURE_LIMIT, 1, "Max Temperature Limit")
time.sleep(1)
getSetting(SMS_STS_MAX_VOLTAGE_LIMIT, 1, "Max Voltage Limit")
time.sleep(1)
getSetting(SMS_STS_MIN_VOLTAGE_LIMIT, 1, "Min Voltage Limit")
time.sleep(1)
getSetting(SMS_STS_UNLOADING_CONDITION, 1, "Unloading Condition")
time.sleep(1)
getSetting(SMS_STS_LED_ALARM_CONDITION, 1, "LED Alarm Condition")
time.sleep(1)
getSetting(SMS_STS_CW_DEAD, 1, "CW Insensitive Zone")
time.sleep(1)
getSetting(SMS_STS_CCW_DEAD, 1, "CCW Insensitive Zone")
time.sleep(1)

print("Servo Settings Readback Complete\n")
time.sleep(5)

#-----------------------------------------------------ENABLE TORQUE------------------------------------------------------------

scs_comm_result, scs_error = packetHandler.write1ByteTxRx(SCS_ID, SMS_STS_TORQUE_ENABLE, 1)
if scs_comm_result != COMM_SUCCESS:
    print(f"[Torque Enable] {packetHandler.getTxRxResult(scs_comm_result)}")
elif scs_error != 0:
    print(f"[Torque Enable] {packetHandler.getRxPacketError(scs_error)}")
else:
    print("[Torque Enable] Torque enabled")
time.sleep(1)

value, scs_comm_result, scs_error = packetHandler.read1ByteTxRx(SCS_ID, SMS_STS_MODE)
if scs_comm_result != COMM_SUCCESS:
    print(f"[Mode] {packetHandler.getTxRxResult(scs_comm_result)}")
elif scs_error != 0:
    print(f"[Mode] {packetHandler.getRxPacketError(scs_error)}")
elif value == 0:
    print(f"[Mode] Current mode: Position Control ({value})")
elif value == 1:
    print(f"[Mode] Current mode: Speed Control ({value})")
elif value == 2:
    print(f"[Mode] Current mode: PWM open-loop speed regulation mode ({value})")
elif value == 3:
    print(f"[Mode] Current mode: Step Servo Mode ({value})")
else:
    print(f"Invalid Mode Value Detected: {value}")

#-----------------------------------------------------POSITION CALIBRATION-----------------------------------------------------

randomStartPos = random.randint(0, 4095)
print(f"Random Start Position: {randomStartPos}")
time.sleep(1)

scs_comm_result, scs_error = packetHandler.WritePosEx(SCS_ID, randomStartPos, SCS_MOVING_SPEED, SCS_MOVING_ACC)
if scs_comm_result != COMM_SUCCESS:
    print(f"[Move to Random Start] {packetHandler.getTxRxResult(scs_comm_result)}")
elif scs_error != 0:
    print(f"[Move to Random Start] {packetHandler.getRxPacketError(scs_error)}")
else:
    print("[Move to Random Start] Reached random start position")

while True:
    scs_present_position, scs_comm_result, scs_error = packetHandler.ReadPos(SCS_ID)
    if scs_comm_result != COMM_SUCCESS:
        print(f"[Move to Random Start] {packetHandler.getTxRxResult(scs_comm_result)}")
    elif scs_error != 0:
        print(f"[Move to Random Start] {packetHandler.getRxPacketError(scs_error)}")
    else:
        print(f"[Move to Random Start] Present position: {scs_present_position}")

    moving, scs_comm_result, scs_error = packetHandler.ReadMoving(SCS_ID)
    if scs_comm_result != COMM_SUCCESS:
        print(f"[Move to Random Start] {packetHandler.getTxRxResult(scs_comm_result)}")

    if moving == 0:
        break

time.sleep(5)



# Read and print the servo's current (true) position
scs_present_position, scs_comm_result, scs_error = packetHandler.ReadPos(SCS_ID)
if scs_comm_result != COMM_SUCCESS:
    print(f"[Read Position] {packetHandler.getTxRxResult(scs_comm_result)}")
elif scs_error != 0:
    print(f"[Read Position] {packetHandler.getRxPacketError(scs_error)}")
else:
    print(f"[Read Position] Current position: {scs_present_position}")

time.sleep(5)

# Move to true zero position using position control
scs_comm_result, scs_error = packetHandler.WritePosEx(SCS_ID, 2048, SCS_MOVING_SPEED, SCS_MOVING_ACC)
if scs_comm_result != COMM_SUCCESS:
    print(f"[Move to Zero] {packetHandler.getTxRxResult(scs_comm_result)}")
elif scs_error != 0:
    print(f"[Move to Zero] {packetHandler.getRxPacketError(scs_error)}")
else:
    print("[Move to Zero] Reached zero position")

while True:
    scs_present_position, scs_comm_result, scs_error = packetHandler.ReadPos(SCS_ID)
    if scs_comm_result != COMM_SUCCESS:
        print(f"[Move to Zero] {packetHandler.getTxRxResult(scs_comm_result)}")
    elif scs_error != 0:
        print(f"[Move to Zero] {packetHandler.getRxPacketError(scs_error)}")
    else:
        print(f"[Move to Zero] Present position: {scs_present_position}")

    moving, scs_comm_result, scs_error = packetHandler.ReadMoving(SCS_ID)
    if scs_comm_result != COMM_SUCCESS:
        print(f"[Move to Zero] {packetHandler.getTxRxResult(scs_comm_result)}")

    if moving == 0:
        break

time.sleep(5)

scs_present_position, scs_comm_result, scs_error = packetHandler.ReadPos(SCS_ID)
if scs_comm_result != COMM_SUCCESS:
    print(f"[Reread Position] {packetHandler.getTxRxResult(scs_comm_result)}")
elif scs_error != 0:
    print(f"[Reread Position] {packetHandler.getRxPacketError(scs_error)}")
else:
    print(f"[Reread Position] Current position: {scs_present_position}")

time.sleep(5)

value, scs_comm_result, scs_error = packetHandler.read1ByteTxRx(SCS_ID, SMS_STS_MODE)
if scs_comm_result != COMM_SUCCESS:
    print(f"[Mode] {packetHandler.getTxRxResult(scs_comm_result)}")
elif scs_error != 0:
    print(f"[Mode] {packetHandler.getRxPacketError(scs_error)}")
elif value == 0:
    print(f"[Mode] Current mode: Position Control ({value})")
elif value == 1:
    print(f"[Mode] Current mode: Speed Control ({value})")
elif value == 2:
    print(f"[Mode] Current mode: PWM open-loop speed regulation mode ({value})")
elif value == 3:
    print(f"[Mode] Current mode: Step Servo Mode ({value})")
else:
    print(f"Invalid Mode Value Detected: {value}")

#-----------------------------------------------------YOUR CODE HERE-----------------------------------------------------------

try:
    print("Starting the script...")
    pass

#-----------------------------------------------------ENDING THE SCRIPT-------------------------------------------------------

finally:
    # Close port
    portHandler.closePort()
    print("Port Closed")