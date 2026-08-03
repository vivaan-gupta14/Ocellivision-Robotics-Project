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
SCS_ID                          = 6                                 # SC Servo ID :
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

SCS_MOVING_SPEED                = 200                               # SC Servo position-mode travel speed (steps/sec) - kept SLOW for range-finding
SCS_MOVING_ACC                  = 10                                # SC Servo moving acc (10 steps/sec^2) - kept SLOW for range-finding

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

# This joint is driven by a lead screw, so it needs several full servo revolutions to
# cover its prismatic travel - not just the single ~360 degree turn that Min/Max_Angle_Limit
# restricts position-mode moves to by default (or from a prior calibration run leaving a
# single-turn range in EEPROM). Setting both limits to 0 puts the servo in multi-turn mode,
# letting WritePosEx goal positions run up to ~+/-32766 (~8 revolutions) instead of clipping
# at +/-4095. Only these two registers are touched - no other EEPROM setting is written here.
packetHandler.unLockEprom(SCS_ID)
writeSetting(SMS_STS_MIN_ANGLE_LIMIT_L, 0, 2, "Min Angle Limit", signed=True)
writeSetting(SMS_STS_MAX_ANGLE_LIMIT_L, 0, 2, "Max Angle Limit", signed=True)
packetHandler.LockEprom(SCS_ID)

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
    Returns 'LEFT', 'RIGHT', 'SPACE', 'ESC', or None if nothing arrived within timeout.

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
    if ch == ' ':
        return 'SPACE'
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

STATUS_HEADER = "TARGET\tPOSITION(raw)\tPOSITION(unwrapped)\tREVS\tSPEED\tLOAD_RAW\tVOLTAGE(dV)\tTEMP(C)\tCURRENT(mA)\tMOVING\tSTATUS_RAW"

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

class Aborted(Exception):
    """Raised when the user presses ESC mid-jog to bail out of the calibration."""

# Present_Position wraps every mechanical revolution (it comes back as a plain 0..4095-ish
# reading no matter how many full turns the lead screw has made), so it alone can't tell a
# position 0 revolutions back from one 12 revolutions back. COUNTS_PER_REV matches this
# file's own "4096 steps = 360 degrees" convention (see SCS_MINIMUM/MAXIMUM_STEP_VALUE above).
COUNTS_PER_REV = 4096

class PositionUnwrapper:
    """Turns the wrapping raw Present_Position readback into a continuous absolute count
    by tracking revolution wraps across successive polls. Assumes consecutive polls
    (POLL_INTERVAL apart) never move more than half a revolution, so any jump bigger than
    that between reads is a wrap, not real motion.
    """
    def __init__(self, initial_raw):
        self.last_raw = initial_raw
        self.unwrapped = initial_raw

    def update(self, raw):
        delta = raw - self.last_raw
        if delta > COUNTS_PER_REV / 2:
            delta -= COUNTS_PER_REV
        elif delta < -COUNTS_PER_REV / 2:
            delta += COUNTS_PER_REV
        self.unwrapped += delta
        self.last_raw = raw
        return self.unwrapped

def jog_until_space(direction_label, target_position, unwrapper):
    """Lets the user hold LEFT/RIGHT to jog the joint (auto-repeat while held) and
    press SPACE to lock in the current actual position. Prints continuously in the
    same free-jog style used elsewhere in this file. Raises Aborted on ESC.
    Returns the locked-in actual position, unwrapped across revolutions.
    """
    print(f"\r\nHold LEFT/RIGHT to drive to the {direction_label}, then press SPACE to lock it in (ESC aborts).", end="\r\n")
    print(STATUS_HEADER, end="\r\n")

    while True:
        # Read the joint's actual position BEFORE handling a key so a new goal can be
        # anchored to where the joint really is right now (see below).
        status = read_status()
        if status is not None:
            unwrapped_position = unwrapper.update(status['position'])

        key = read_key(POLL_INTERVAL)

        if key == 'ESC':
            raise Aborted("Aborted by user during jog.")
        elif key in ('LEFT', 'RIGHT'):
            step = -SCS_POSITION_STEP if key == 'LEFT' else SCS_POSITION_STEP
            # Anchor the new goal to the joint's actual current position rather than
            # accumulating on top of the last commanded target: keyboard auto-repeat fires
            # every ~30-50ms while a key is held, far faster than SCS_MOVING_SPEED (kept
            # slow on purpose for range-finding) can physically cover one SCS_POSITION_STEP.
            # An accumulating target ran way ahead of the servo's actual position, so on
            # release the servo kept driving for seconds to work off that backlog - looking
            # like it was "still moving" even though no more keys were being read.
            anchor = unwrapped_position if status is not None else target_position
            target_position = anchor + step

            scs_comm_result, scs_error = packetHandler.WritePosEx(SCS_ID, target_position, SCS_MOVING_SPEED, SCS_MOVING_ACC)
            if scs_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(scs_comm_result), end="\r\n")
            elif scs_error != 0:
                print("%s" % packetHandler.getRxPacketError(scs_error), end="\r\n")

        if status is None:
            print(f"{target_position}\t[status read failed]", end="\r\n")
        else:
            revolutions = unwrapped_position / COUNTS_PER_REV
            print(
                f"{target_position}\t{status['position']}\t{unwrapped_position}\t{revolutions:.2f}\t{status['speed']}\t{status['load_raw']}\t"
                f"{status['voltage_dV']}\t{status['temperature_C']}\t{status['current_mA']}\t"
                f"{status['moving']}\t{status['status_raw']}",
                end="\r\n",
            )

        if key == 'SPACE':
            locked_position = unwrapped_position if status is not None else target_position
            print(f"--- Locked in {direction_label}: {locked_position} ({locked_position / COUNTS_PER_REV:.2f} revs) ---", end="\r\n")
            return locked_position

def drive_to_center(goal_position, unwrapper, tolerance=3):
    """Blocking move to goal_position, printing continuously in the same
    [ID] GoalPos/PresPos/PresSpd style as positionCalibration.move_to_position.
    goal_position and the printed present position are both in the same unwrapped
    (revolution-aware) frame as jog_until_space's return values."""
    scs_comm_result, scs_error = packetHandler.WritePosEx(SCS_ID, goal_position, SCS_MOVING_SPEED, SCS_MOVING_ACC)
    if scs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(scs_comm_result), end="\r\n")
    elif scs_error != 0:
        print("%s" % packetHandler.getRxPacketError(scs_error), end="\r\n")

    has_moved = False
    while True:
        scs_present_position, scs_present_speed, scs_comm_result, scs_error = packetHandler.ReadPosSpeed(SCS_ID)
        if scs_comm_result != COMM_SUCCESS:
            print(packetHandler.getTxRxResult(scs_comm_result), end="\r\n")
        else:
            unwrapped_position = unwrapper.update(scs_present_position)
            print("[ID:%03d] GoalPos:%d PresPos(raw):%d PresPos(unwrapped):%d PresSpd:%d" % (SCS_ID, goal_position, scs_present_position, unwrapped_position, scs_present_speed), end="\r\n")
        if scs_error != 0:
            print(packetHandler.getRxPacketError(scs_error), end="\r\n")

        moving, scs_comm_result, scs_error = packetHandler.ReadMoving(SCS_ID)
        if scs_comm_result != COMM_SUCCESS:
            print(packetHandler.getTxRxResult(scs_comm_result), end="\r\n")

        if moving == 1:
            has_moved = True

        # Trust moving==0 as "arrived" once we've seen it move, OR if we're already
        # within tolerance (the servo may never set moving==1 if already close enough).
        if moving == 0 and (has_moved or abs(unwrapped_position - goal_position) <= tolerance):
            break

try:
    print("Starting the script...")

    if os.name == 'nt':
        raise RuntimeError("This arrow-key hold-control script requires Mac/Linux (uses select() on stdin).")

    tty.setraw(fd)  # raw mode so arrow-key escape sequences arrive unbuffered
    # setraw() disables output post-processing, so plain "\n" no longer returns the
    # cursor to column 0 - every print below must end with "\r\n" instead.

    initial_position, _, scs_comm_result, scs_error = packetHandler.ReadPosSpeed(SCS_ID)
    if scs_comm_result != COMM_SUCCESS or scs_error != 0:
        initial_position = 0
    print(f"Starting raw position: {initial_position}", end="\r\n")
    print("This will find the raw center and range of the prismatic joint.", end="\r\n")
    print("Step 1: jog to the furthest BACK (retracted) position.", end="\r\n")

    # Present_Position wraps every revolution, so all positions from here on are tracked
    # through a single PositionUnwrapper instance (started at 0 revolutions offset from
    # wherever the joint happens to be right now) to get a continuous, revolution-aware count.
    unwrapper = PositionUnwrapper(initial_position)

    back_position = jog_until_space("furthest BACK (retracted) position", initial_position, unwrapper)

    print("\r\nStep 2: jog to the furthest FORWARD (extended) position.", end="\r\n")
    forward_position = jog_until_space("furthest FORWARD (extended) position", back_position, unwrapper)

    raw_center = round((back_position + forward_position) / 2)
    back_range = abs(raw_center - back_position)      # steps from center to the BACK limit
    forward_range = abs(forward_position - raw_center) # steps from center to the FORWARD limit

    def print_summary():
        print(f"Furthest BACK position (unwrapped):     {back_position}\t({back_position / COUNTS_PER_REV:.2f} revs)", end="\r\n")
        print(f"Furthest FORWARD position (unwrapped):  {forward_position}\t({forward_position / COUNTS_PER_REV:.2f} revs)", end="\r\n")
        print(f"Computed CENTER position (unwrapped):   {raw_center}\t({raw_center / COUNTS_PER_REV:.2f} revs)", end="\r\n")
        print(f"Range BACK of center (steps):            {back_range}\t({back_range / COUNTS_PER_REV:.2f} revs)", end="\r\n")
        print(f"Range FORWARD of center (steps):         {forward_range}\t({forward_range / COUNTS_PER_REV:.2f} revs)", end="\r\n")

    print("\r\n--- Jog Complete ---", end="\r\n")
    print_summary()
    print("Sleeping 5 seconds before driving to center...", end="\r\n")
    time.sleep(5)

    print(f"\r\nDriving to computed center position: {raw_center}", end="\r\n")
    drive_to_center(raw_center, unwrapper)

    print("\r\n=== Prismatic Joint Raw Calibration Summary ===", end="\r\n")
    print_summary()

#--------------------------------------ENDING THE SCRIPT-------------------------------------------------------

except Aborted as e:
    print(f"\r\n{e}", end="\r\n")

finally:
    if os.name != 'nt':
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    # Close port
    portHandler.closePort()
    print("Port Closed")
