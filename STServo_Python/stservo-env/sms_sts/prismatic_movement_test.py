#--------------------------------------GENERAL IMPORTS----------------------------------------------------------

import sys
import os
import time
import select
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
SCS_MINIMUM_STEP_VALUE          = -4095                             # 4096 steps = 360 degrees. Minimum value is -4095 (~ 1 revolutions)
SCS_MAXIMUM_STEP_VALUE          = 4095                              # 4096 steps = 360 degrees. Maximum value is 4095 (~ 1 revolutions)
                                                                    #Set both values to 0 for unrestricted rotation

SCS_MAXIMUM_TEMPERATURE         = 55                                # SC Servo maximum temperature (55 degrees Celsius)

SCS_MAXIMUM_INPUT_VOLTAGE       = 130                               #13.0 Volts
SCS_MINIMUM_INPUT_VOLTAGE       = 90                                #9.0 Volts

SCS_UNLOADING_CONDITIONS        = 47                                # 0b101111: bit0 Voltage, bit1 Sensor, bit2 Temperature,
                                                                     # bit3 Current, bit4 Angle, bit5 Overload -- Angle (bit4) disabled
                                                                     # so hitting the angle limit no longer cuts torque
SCS_LED_ALARM_CONDITIONS        = 63                                # Same bit layout as above; LED still flashes for all 6 conditions,
                                                                     # including Angle, so the limit is still visible

SCS_MOVING_SPEED                = 2400                              # SC Servo position-mode travel speed (steps/sec)
SCS_MOVING_ACC                  = 50                                # SC Servo moving acc (50 steps/sec^2)

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
    print("Succeeded to change the baudrate")
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

def writeSetting(address, value, num_bytes, name, signed=False):
    """signed=True sign-magnitude-encodes the value (bit 15 = sign) before writing.
    write2ByteTxRx packs the raw Python int as plain two's-complement bytes, but these
    registers (e.g. angle limits) use the same sign-magnitude format as Present_Position,
    so negative values must be converted with scs_toscs first or they land wrong on the
    servo even though the write ACKs."""
    if num_bytes == 1:
        scs_comm_result, scs_error = packetHandler.write1ByteTxRx(SCS_ID, address, value)
    else:
        wire_value = packetHandler.scs_toscs(value, 15) if signed else value
        scs_comm_result, scs_error = packetHandler.write2ByteTxRx(SCS_ID, address, wire_value)

    if scs_comm_result != COMM_SUCCESS:
        print(f"[{name}] {packetHandler.getTxRxResult(scs_comm_result)}")
    elif scs_error != 0:
        print(f"[{name}] {packetHandler.getRxPacketError(scs_error)}")
    else:
        print(f"[{name}] Set to {value}")

def readSetting(address, num_bytes, name, signed=False):
    """Reads back a setting directly from the servo so it can be compared against the
    value we just wrote - the write can ACK even if the servo clamps/rejects the value.

    signed=True decodes the raw word as sign-magnitude (bit 15 = sign), matching how
    Present_Position is decoded elsewhere in this script - the angle limit registers
    use the same encoding, whereas write2ByteTxRx sends negative Python ints as plain
    two's-complement bytes, so the raw word alone can look wrong even when correctly set.
    """
    if num_bytes == 1:
        value, scs_comm_result, scs_error = packetHandler.read1ByteTxRx(SCS_ID, address)
    else:
        value, scs_comm_result, scs_error = packetHandler.read2ByteTxRx(SCS_ID, address)

    if scs_comm_result != COMM_SUCCESS:
        print(f"[{name}] Readback failed: {packetHandler.getTxRxResult(scs_comm_result)}")
    elif scs_error != 0:
        print(f"[{name}] Readback failed: {packetHandler.getRxPacketError(scs_error)}")
    elif signed:
        print(f"[{name}] Readback = {value} (raw), {packetHandler.scs_tohost(value, 15)} (signed)")
    else:
        print(f"[{name}] Readback = {value}")

# EEPROM must be unlocked before these addresses can be written
packetHandler.unLockEprom(SCS_ID)

writeSetting(SMS_STS_MIN_ANGLE_LIMIT_L, SCS_MINIMUM_STEP_VALUE, 2, "Min Angle Limit", signed=True)
writeSetting(SMS_STS_MAX_ANGLE_LIMIT_L, SCS_MAXIMUM_STEP_VALUE, 2, "Max Angle Limit", signed=True)
writeSetting(SMS_STS_MAX_TEMPERATURE_LIMIT, SCS_MAXIMUM_TEMPERATURE, 1, "Max Temperature Limit")
writeSetting(SMS_STS_MAX_VOLTAGE_LIMIT, SCS_MAXIMUM_INPUT_VOLTAGE, 1, "Max Voltage Limit")
writeSetting(SMS_STS_MIN_VOLTAGE_LIMIT, SCS_MINIMUM_INPUT_VOLTAGE, 1, "Min Voltage Limit")
writeSetting(SMS_STS_UNLOADING_CONDITION, SCS_UNLOADING_CONDITIONS, 1, "Unloading Condition")
writeSetting(SMS_STS_LED_ALARM_CONDITION, SCS_LED_ALARM_CONDITIONS, 1, "LED Alarm Condition")
writeSetting(SMS_STS_CW_DEAD, SCS_CW_INSENSITIVE_ZONE, 1, "CW Insensitive Zone")
writeSetting(SMS_STS_CCW_DEAD, SCS_CCW_INSENSITIVE_ZONE, 1, "CCW Insensitive Zone")

# Re-lock EEPROM to protect the settings just written
packetHandler.LockEprom(SCS_ID)

# Diagnostic: check whether torque is actually enabled on the servo (register 40).
# WritePosEx never writes this register, so if it's 0 the servo will ACK position
# commands over serial but the motor will never actually be driven.
torque_enabled, scs_comm_result, scs_error = packetHandler.read1ByteTxRx(SCS_ID, SMS_STS_TORQUE_ENABLE)
if scs_comm_result != COMM_SUCCESS:
    print(f"[Torque Enable Check] {packetHandler.getTxRxResult(scs_comm_result)}")
elif scs_error != 0:
    print(f"[Torque Enable Check] {packetHandler.getRxPacketError(scs_error)}")
else:
    print(f"[Torque Enable Check] Torque_Enable register = {torque_enabled}")

# Read back every setting just written to confirm what the servo actually stored -
# a write can ACK even if the servo clamps or rejects the requested value.
readSetting(SMS_STS_MIN_ANGLE_LIMIT_L, 2, "Min Angle Limit", signed=True)
readSetting(SMS_STS_MAX_ANGLE_LIMIT_L, 2, "Max Angle Limit", signed=True)
readSetting(SMS_STS_MAX_TEMPERATURE_LIMIT, 1, "Max Temperature Limit")
readSetting(SMS_STS_MAX_VOLTAGE_LIMIT, 1, "Max Voltage Limit")
readSetting(SMS_STS_MIN_VOLTAGE_LIMIT, 1, "Min Voltage Limit")
readSetting(SMS_STS_UNLOADING_CONDITION, 1, "Unloading Condition")
readSetting(SMS_STS_LED_ALARM_CONDITION, 1, "LED Alarm Condition")
readSetting(SMS_STS_CW_DEAD, 1, "CW Insensitive Zone")
readSetting(SMS_STS_CCW_DEAD, 1, "CCW Insensitive Zone")

time.sleep(10)

print("Servo Settings Applied")

#--------------------------------------YOUR CODE HERE-----------------------------------------------------------

# A terminal only reports key-presses, never key-releases, so "holding" a key is detected
# via the OS's keyboard auto-repeat: while physically held, the same arrow-key escape
# sequence keeps arriving every ~30-50ms, and each arrival nudges the goal position one
# more SCS_POSITION_STEP in that direction.
POLL_INTERVAL      = 0.02   # seconds between non-blocking stdin polls
SCS_POSITION_STEP  = 40     # steps added to the goal position per key repeat while held

ESCAPE_SEQUENCE_TIMEOUT = 0.1  # time to wait for the rest of an arrow-key escape sequence

def read_key(timeout):
    """Non-blocking read of a single key event from stdin.
    Returns 'LEFT', 'RIGHT', 'ESC', or None if nothing arrived within timeout.

    Reads directly from the raw file descriptor (os.read) rather than the
    buffered sys.stdin: a buffered read can silently slurp the whole arrow-key
    escape sequence into its internal buffer on the first call, leaving
    nothing at the OS level for the next select() to see - making a real
    arrow press look like a lone ESC.
    """
    ready, _, _ = select.select([fd], [], [], timeout)
    if not ready:
        return None
    ch = os.read(fd, 1).decode()
    if ch != '\x1b':
        return None
    # Possible arrow-key escape sequence: ESC [ C/D  or  ESC O C/D
    ready, _, _ = select.select([fd], [], [], ESCAPE_SEQUENCE_TIMEOUT)
    if not ready:
        return 'ESC'
    ch2 = os.read(fd, 1).decode()
    if ch2 not in ('[', 'O'):
        return None
    ready, _, _ = select.select([fd], [], [], ESCAPE_SEQUENCE_TIMEOUT)
    if not ready:
        return None
    ch3 = os.read(fd, 1).decode()
    if ch3 == 'C':
        return 'RIGHT'
    elif ch3 == 'D':
        return 'LEFT'
    return None

# Present_Position(56) .. Present_Current(70) are contiguous in the memory table, so one raw
# read grabs every telemetry field in a single serial transaction.
STATUS_READ_LENGTH = 15

STATUS_HEADER = "TARGET\tPOSITION\tSPEED\tLOAD_RAW\tVOLTAGE(dV)\tTEMP(C)\tCURRENT(mA)\tMOVING\tSTATUS_RAW"

def read_status():
    """Non-blocking-free single read of every present-value register. Returns a dict, or None on comm error."""
    data, scs_comm_result, scs_error = packetHandler.readTxRx(SCS_ID, SMS_STS_PRESENT_POSITION_L, STATUS_READ_LENGTH)
    if scs_comm_result != COMM_SUCCESS or scs_error != 0 or len(data) < STATUS_READ_LENGTH:
        return None

    def word(offset):
        return packetHandler.scs_makeword(data[offset], data[offset + 1])

    return {
        "position": packetHandler.scs_tohost(word(0), 15),
        "speed": packetHandler.scs_tohost(word(2), 15),
        "load_raw": word(4),
        "voltage_dV": data[6],
        "temperature_C": data[7],
        "status_raw": data[9],
        "moving": data[10],
        "current_mA": packetHandler.scs_tohost(word(13), 15),
    }

try:
    print("Starting the script...")

    if os.name == 'nt':
        raise RuntimeError("This arrow-key hold-control script requires Mac/Linux (uses select() on stdin).")

    target_position = 0
    scs_comm_result, scs_error = packetHandler.WritePosEx(SCS_ID, target_position, SCS_MOVING_SPEED, SCS_MOVING_ACC)
    if scs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(scs_comm_result))
    elif scs_error != 0:
        print("%s" % packetHandler.getRxPacketError(scs_error))
    print(f"Starting position: {target_position}")
    time.sleep(5)
    print("Hold LEFT/RIGHT arrow to move the servo, press ESC to quit.")
    print(STATUS_HEADER)

    tty.setraw(fd)  # raw mode so arrow-key escape sequences arrive unbuffered
    # setraw() disables output post-processing, so plain "\n" no longer returns the
    # cursor to column 0 - every print below must end with "\r\n" instead.

    while True:
        key = read_key(POLL_INTERVAL)

        if key == 'ESC':
            break
        elif key in ('LEFT', 'RIGHT'):
            step = -SCS_POSITION_STEP if key == 'LEFT' else SCS_POSITION_STEP
            target_position += step

            scs_comm_result, scs_error = packetHandler.WritePosEx(SCS_ID, target_position, SCS_MOVING_SPEED, SCS_MOVING_ACC)
            if scs_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(scs_comm_result), end="\r\n")
            elif scs_error != 0:
                print("%s" % packetHandler.getRxPacketError(scs_error), end="\r\n")

        status = read_status()
        if status is None:
            print(f"{target_position}\t[status read failed]", end="\r\n")
        else:
            print(
                f"{target_position}\t{status['position']}\t{status['speed']}\t{status['load_raw']}\t"
                f"{status['voltage_dV']}\t{status['temperature_C']}\t{status['current_mA']}\t"
                f"{status['moving']}\t{status['status_raw']}",
                end="\r\n",
            )

#--------------------------------------ENDING THE SCRIPT-------------------------------------------------------

finally:
    if os.name != 'nt':
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    # Close port
    portHandler.closePort()
    print("Port Closed")
