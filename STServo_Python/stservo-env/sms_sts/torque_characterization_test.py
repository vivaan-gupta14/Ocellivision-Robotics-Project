
#-----------------------------------------------------GENERAL IMPORTS----------------------------------------------------------

import sys
import os
import time
import csv
import statistics
import matplotlib.pyplot as plt
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

#---------------------------------------------MOVE TO POSITION WITH TELEMETRY LOGGING FUNCTION---------------------------------------------
# Same arrival algorithm as move_to_position() above (has_moved + SCS_POSITION_TOLERANCE dead-zone
# check), but every poll iteration also reads full telemetry (position/speed/torque/current/voltage/
# temperature/moving) in one combined transaction and logs it, instead of just printing position/speed.

def move_to_position_with_logging(scs_id, goal_position, moving_speed, moving_acc,
                                   phase, cycle, mass_g, abort_current_ma,
                                   csv_writer, csv_file, samples, t0):
    scs_comm_result, scs_error = packetHandler.WritePosEx(scs_id, goal_position, moving_speed, moving_acc)
    if scs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(scs_comm_result))
    elif scs_error != 0:
        print("%s" % packetHandler.getRxPacketError(scs_error))

    has_moved = False
    while 1:
        sample_t0 = time.time()

        telemetry = sample_and_log(scs_id, phase, goal_position, mass_g, abort_current_ma,
                                    csv_writer, csv_file, samples, cycle, t0)

        if telemetry is not None:
            if telemetry["moving"] == 1:
                has_moved = True

            if telemetry["moving"] == 0 and (has_moved or abs(telemetry["present_position"] - goal_position) <= SCS_POSITION_TOLERANCE):
                break

        dt = time.time() - sample_t0
        if dt < SAMPLE_PERIOD:
            time.sleep(SAMPLE_PERIOD - dt)


def hold_with_logging(scs_id, duration, phase, goal_position, cycle, mass_g, abort_current_ma,
                       csv_writer, csv_file, samples, t0):
    hold_start = time.time()
    while time.time() - hold_start < duration:
        sample_t0 = time.time()
        sample_and_log(scs_id, phase, goal_position, mass_g, abort_current_ma,
                        csv_writer, csv_file, samples, cycle, t0)
        dt = time.time() - sample_t0
        if dt < SAMPLE_PERIOD:
            time.sleep(SAMPLE_PERIOD - dt)


#-----------------------------------------------------TELEMETRY SAMPLING-----------------------------------------------------------
# Present_Position(56)..Present_Current(70) are contiguous in the memory table, so one raw read
# grabs position/speed/torque(load)/voltage/temperature/moving/current in a single serial
# transaction - this is what makes reliable SAMPLE_HZ sampling of every field possible.

class AbortExperiment(Exception):
    """Raised to unwind cleanly to the safety shutdown path in main()."""


def read_telemetry(scs_id):
    data, comm_result, error = packetHandler.readTxRx(scs_id, SMS_STS_PRESENT_POSITION_L, 15)
    if comm_result != COMM_SUCCESS or len(data) < 15:
        return None, comm_result, error

    def word(offset):
        return packetHandler.scs_makeword(data[offset], data[offset + 1])

    telemetry = {
        "present_position": packetHandler.scs_tohost(word(0), 15),
        "present_speed": packetHandler.scs_tohost(word(2), 15),
        "present_torque_pct": packetHandler.scs_tohost(word(4), 10) / 10.0,  # 0.1%-of-max-torque units, bit10 = direction
        "present_voltage": data[6] / 10.0,                                  # 0.1V units
        "present_temperature": data[7],
        "status_reg_raw": data[9],
        "moving": data[10],
        "present_current_ma": packetHandler.scs_tohost(word(13), 15),
    }
    return telemetry, comm_result, error


def sample_and_log(scs_id, phase, goal_position, mass_g, abort_current_ma,
                    csv_writer, csv_file, samples, cycle, t0):
    telemetry, comm_result, error = read_telemetry(scs_id)
    if telemetry is None:
        print(f"  [WARN] telemetry read failed ({packetHandler.getTxRxResult(comm_result)}), retrying...")
        return None

    now = time.time()
    row = {
        "timestamp_ms": int(now * 1000),
        "elapsed_s": round(now - t0, 3),
        "cycle": cycle,
        "phase": phase,
        "goal_position": goal_position,
        "present_position": telemetry["present_position"],
        "present_speed": telemetry["present_speed"],
        "commanded_speed": SCS_MOVING_SPEED,
        "commanded_acc": SCS_MOVING_ACC,
        "present_torque_pct": telemetry["present_torque_pct"],
        "present_current_ma": telemetry["present_current_ma"],
        "present_voltage": telemetry["present_voltage"],
        "present_temperature": telemetry["present_temperature"],
        "moving": telemetry["moving"],
        "status_reg_raw": telemetry["status_reg_raw"],
        "packet_error_flags": error,
        "mass_g": mass_g,
    }
    csv_writer.writerow([row[col] for col in CSV_COLUMNS])
    csv_file.flush()
    samples.append(row)

    print(f"  cycle {cycle:2d} {phase:12s} goal={goal_position:5d} pos={telemetry['present_position']:5d} "
          f"torque={telemetry['present_torque_pct']:6.1f}% cur={telemetry['present_current_ma']:5d}mA "
          f"spd={telemetry['present_speed']:5d} temp={telemetry['present_temperature']:3d}C "
          f"volt={telemetry['present_voltage']:4.1f}V moving={telemetry['moving']}")

    if abs(telemetry["present_current_ma"]) > abort_current_ma:
        raise AbortExperiment(
            f"present current {telemetry['present_current_ma']}mA exceeded abort threshold "
            f"{abort_current_ma}mA during {phase} (cycle {cycle})")
    if error & FAULT_ERRBITS:
        raise AbortExperiment(
            f"servo fault flags during {phase} (cycle {cycle}): {packetHandler.getRxPacketError(error)}")

    return telemetry

#-------------------------------------------------------INITIALIZE SERVO SETTINGS-------------------------------------------------

# Default setting
SCS_ID                          = 9                                 # SC Servo ID : 10
BAUDRATE                        = 1000000                           # SC Servo default baudrate : 1000000 (1 Mbps)
DEVICENAME                      = '/dev/tty.usbmodem5AAF2197651'    # Check which port is being used on your controller
                                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
SCS_MOVING_SPEED                = 1200                              # SC Servo moving speed (2400 steps/sec)
SCS_MOVING_ACC                  = 10                                # SC Servo moving acc (50 steps/sec^2)

SCS_POSITION_TOLERANCE          = 3                                 # Dead-zone: positions within this range count as "arrived"
DESIRED_CENTER_RAW              = 3385                              # The raw position that should become the new center
RANGE                           = 800                               # The maximum offset from the center
CORRECTION                      = DESIRED_CENTER_RAW - 2048         # The position correction to be applied
SCS_CALIBRATED_CENTER           = 2048                              # The position value that becomes the "center" position after correction
SCS_MIN_POSITION                = SCS_CALIBRATED_CENTER - RANGE     # The minimum position value after correction
SCS_MAX_POSITION                = SCS_CALIBRATED_CENTER + RANGE     # The maximum position value after correction
HARD_ANGLE_LIMIT_MARGIN         = 50

MAX_TEMP                        = 55                                # Maxmimum temperature in degrees celcius
MAX_INPUT_VOLTAGE               = 130                               # Maximum input voltage in 0.1V units
MIN_INPUT_VOLTAGE               = 90                                # Minimum input voltage in 0.1V units
CW_INSENSITIVE_ZONE             = 1                                 # Clockwise insensitive zone in steps
CCW_INSENSITIVE_ZONE            = 1                                 # Counterclockwise insensitive zone in steps

# --------------------------------------TORQUE TEST SPECIFIC SETTINGS--------------------------------------------

NUM_CYCLES                      = 5                                 # Number of min<->max cycles to run
HOLD_SECONDS                    = 2.0                                # Dwell time at each end of travel, seconds
SAMPLE_HZ                       = 30                                 # Telemetry sampling rate during motion/hold
SAMPLE_PERIOD                   = 1.0 / SAMPLE_HZ

# Conservative runtime torque cap applied via SRAM Torque_Limit (address 48) for the whole test, as
# a safety backstop independent of whatever final EPROM protection values this experiment informs.
# 0-1000 = 0.0%-100.0% of max torque, same units as SCS_TORQUE_LIMIT in template.py.
TEST_TORQUE_LIMIT               = 250                                # 25.0% of max force
DEFAULT_ABORT_CURRENT_MA        = 900                                 # Auto-abort if present current exceeds this many mA

# EEPROM Addresses not already defined in scservo_sdk/sms_sts.py's control table
SMS_STS_MAX_TEMPERATURE_LIMIT   = 13
SMS_STS_MAX_VOLTAGE_LIMIT       = 14
SMS_STS_MIN_VOLTAGE_LIMIT       = 15
SMS_STS_UNLOADING_CONDITION     = 19
SMS_STS_LED_ALARM_CONDITION     = 20

# SRAM, 2 bytes, 0.1%/count - runtime torque cap. Not exposed by name in this SDK, but matches the
# standard FeeTech STS/SMS memory table (same table that gives MIN_ANGLE_LIMIT_L=9, CW_DEAD=26,
# OFS_L=31, MODE=33, TORQUE_ENABLE=40, PRESENT_CURRENT_L=69, ... all already defined by this SDK).
STS_TORQUE_LIMIT_L              = 48

FAULT_ERRBITS = ERRBIT_VOLTAGE | ERRBIT_OVERHEAT | ERRBIT_OVERELE | ERRBIT_OVERLOAD

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(SCRIPT_DIR, "torque_test_data")

CSV_COLUMNS = [
    "timestamp_ms", "elapsed_s", "cycle", "phase", "goal_position", "present_position",
    "present_speed", "commanded_speed", "commanded_acc", "present_torque_pct",
    "present_current_ma", "present_voltage", "present_temperature", "moving",
    "status_reg_raw", "packet_error_flags", "mass_g",
]

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
print(f"SC Servo Hard Angle Limit Margin: {HARD_ANGLE_LIMIT_MARGIN}")
print(f"SC Servo Maximum Temperature: {MAX_TEMP}")
print(f"SC Servo Maximum Input Voltage: {MAX_INPUT_VOLTAGE}")
print(f"SC Servo Minimum Input Voltage: {MIN_INPUT_VOLTAGE}")
print(f"SC Servo Clockwise Insensitive Zone: {CW_INSENSITIVE_ZONE}")
print(f"SC Servo Counterclockwise Insensitive Zone: {CCW_INSENSITIVE_ZONE}")
print(f"Torque Test Cycles: {NUM_CYCLES}")
print(f"Torque Test Hold Seconds: {HOLD_SECONDS}")
print(f"Torque Test Sample Rate (Hz): {SAMPLE_HZ}")
print(f"Torque Test Runtime Torque Limit: {TEST_TORQUE_LIMIT}")
print(f"Torque Test Abort Current (mA): {DEFAULT_ABORT_CURRENT_MA}")
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

#---------------------------------------------------------SERVO SETTINGS------------------------------------------------------

# EEPROM must be unlocked before these addresses can be written
packetHandler.unLockEprom(SCS_ID)

#----------------------------------Apply Settings

setSetting(address=SMS_STS_MAX_TEMPERATURE_LIMIT, value = MAX_TEMP, num_bytes = 1, name = "Maximum Temperature")
setSetting(address=SMS_STS_MAX_VOLTAGE_LIMIT, value = MAX_INPUT_VOLTAGE, num_bytes = 1, name = "Maximum Input Voltage")
setSetting(address=SMS_STS_MIN_VOLTAGE_LIMIT, value = MIN_INPUT_VOLTAGE, num_bytes = 1, name = "Minimum Input Voltage")
setSetting(address=SMS_STS_CW_DEAD, value = CW_INSENSITIVE_ZONE, num_bytes = 1, name = "Clockwise Insensitive Zone")
setSetting(address=SMS_STS_CCW_DEAD, value = CCW_INSENSITIVE_ZONE, num_bytes = 1, name = "Counterclockwise Insensitive Zone")
setSetting(address=SMS_STS_MAX_ANGLE_LIMIT_L, value = SCS_MAX_POSITION + HARD_ANGLE_LIMIT_MARGIN, num_bytes = 2, name = "Max Hardware Angle Limit")
setSetting(address=SMS_STS_MIN_ANGLE_LIMIT_L, value = SCS_MIN_POSITION - HARD_ANGLE_LIMIT_MARGIN, num_bytes = 2, name = "Min Hardware Angle Limit")

#-----------------------------------------------

# Re-lock EEPROM to protect the settings just written
packetHandler.LockEprom(SCS_ID)

#----------------------------------Verify Settings

getSetting(address=SMS_STS_MAX_TEMPERATURE_LIMIT, num_bytes=1, name="Maximum Temperature")
getSetting(address=SMS_STS_MAX_VOLTAGE_LIMIT, num_bytes=1, name="Maximum Input Voltage")
getSetting(address=SMS_STS_MIN_VOLTAGE_LIMIT, num_bytes=1, name="Minimum Input Voltage")
getSetting(address=SMS_STS_CW_DEAD, num_bytes=1, name="Clockwise Insensitive Zone")
getSetting(address=SMS_STS_CCW_DEAD, num_bytes=1, name="Counterclockwise Insensitive Zone")
getSetting(address=SMS_STS_MAX_ANGLE_LIMIT_L, num_bytes=2, name="Max Hardware Angle Limit")
getSetting(address=SMS_STS_MIN_ANGLE_LIMIT_L, num_bytes=2, name="Min Hardware Angle Limit")

#------------------------------------------------

time.sleep(10)


#-----------------------------------------------------CSV LOGGING SETUP-----------------------------------------------------------

def open_csv(mass_g):
    os.makedirs(DATA_DIR, exist_ok=True)
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    path = os.path.join(DATA_DIR, f"torque_test_{mass_g:g}g_{timestamp}.csv")
    f = open(path, "w", newline="")
    writer = csv.writer(f)
    writer.writerow(CSV_COLUMNS)
    return f, writer, path


#-----------------------------------------------------SUMMARY AND GRAPHING-----------------------------------------------------------

def print_summary(samples):
    if not samples:
        print("\nNo samples were collected.")
        return

    print("\n--- Summary ---")
    by_phase = {}
    for row in samples:
        by_phase.setdefault(row["phase"], []).append(row)

    for phase, rows in by_phase.items():
        currents = [row["present_current_ma"] for row in rows]
        torques = [row["present_torque_pct"] for row in rows]
        print(f"  {phase:14s} mean current: {statistics.mean(currents):7.1f} mA   peak current: {max(currents, key=abs):6.0f} mA   "
              f"mean torque: {statistics.mean(torques):6.1f}%   peak torque: {max(torques, key=abs):6.1f}%   n={len(rows)}")

    all_currents = [row["present_current_ma"] for row in samples]
    all_torques = [row["present_torque_pct"] for row in samples]
    print(f"  {'overall':14s} max |current|: {max(all_currents, key=abs):.0f} mA   max |torque|: {max(all_torques, key=abs):.1f}%")


def plot_telemetry(samples, events, mass_g, csv_path):
    if not samples:
        print("\nNo samples were collected - skipping graph.")
        return

    times = [row["elapsed_s"] for row in samples]
    torques = [row["present_torque_pct"] for row in samples]
    currents = [row["present_current_ma"] for row in samples]

    # Two stacked subplots (shared time axis) rather than a dual-axis chart, since torque (%) and
    # current (mA) are different units/scales.
    fig, (ax_torque, ax_current) = plt.subplots(2, 1, sharex=True, figsize=(11, 7))

    TORQUE_COLOR = "#2a78d6"   # categorical slot 1 (blue)
    CURRENT_COLOR = "#eb6834"  # categorical slot 2 (orange)
    EVENT_COLOR = "#52514e"    # muted ink, text/lines only - never a series color

    ax_torque.plot(times, torques, color=TORQUE_COLOR, linewidth=2)
    ax_torque.set_ylabel("Torque (% of max)")
    ax_torque.axhline(0, color=EVENT_COLOR, linewidth=0.75, alpha=0.4)
    ax_torque.set_title(f"Servo ID {SCS_ID} Torque Characterization - Mass: {mass_g:g} g", pad=14)

    ax_current.plot(times, currents, color=CURRENT_COLOR, linewidth=2)
    ax_current.set_ylabel("Current (mA)")
    ax_current.set_xlabel("Elapsed Time (s)")

    for ax in (ax_torque, ax_current):
        for elapsed_s, label in events:
            ax.axvline(elapsed_s, color=EVENT_COLOR, linewidth=1, linestyle="--", alpha=0.6)
        ax.margins(x=0.01)

    # Only annotate the top subplot with text, so labels aren't drawn twice. Anchored in axes
    # fraction coordinates (not data coordinates) so labels stay inside the plot regardless of
    # the torque range, and never collide with the title above the axes.
    for elapsed_s, label in events:
        ax_torque.annotate(label, xy=(elapsed_s, 0.98), xycoords=("data", "axes fraction"),
                            xytext=(3, -3), textcoords="offset points",
                            fontsize=8, color=EVENT_COLOR, rotation=90, va="top", ha="left")

    fig.tight_layout()

    png_path = os.path.splitext(csv_path)[0] + ".png"
    fig.savefig(png_path, dpi=150)
    print(f"Graph saved to {png_path}")
    plt.show()


def print_safety_reminder():
    print("""
--- SAFETY CHECK ---
Before continuing, confirm:
  - Servo body is rigidly clamped.
  - EndoWrist shaft is rigidly clamped (not held by hand).
  - Hanging mass is attached and free to move vertically.
  - You are ready to hit Ctrl+C immediately if current spikes, the cable
    frays, the instrument binds, the servo stalls, or you hear/see anything
    unusual.
""")
    confirmation = input("Type 'yes' to begin: ").strip().lower()
    if confirmation != "yes":
        raise AbortExperiment("User did not confirm safety checklist.")


#------------------------------------------------------------MAIN CONTROL LOOP------------------------------------------------

samples = []
csv_file = None
csv_path = None
events = []
mass_g = 0.0

try:
    # Ask for the hanging test mass - the only interactive input this script needs.
    while True:
        userInput = input("Enter the hanging test mass in grams: ").strip()
        try:
            mass_g = float(userInput)
            break
        except ValueError:
            print(f"'{userInput}' is not a valid number. Please try again.\n")

    print_safety_reminder()

    csv_file, csv_writer, csv_path = open_csv(mass_g)
    print(f"Logging to {csv_path}")

    # Conservative runtime torque cap + explicit torque enable for the duration of the test.
    setSetting(address=STS_TORQUE_LIMIT_L, value=TEST_TORQUE_LIMIT, num_bytes=2, name="Runtime Torque Limit")
    setSetting(address=SMS_STS_TORQUE_ENABLE, value=1, num_bytes=1, name="Torque Enable")

    t0 = time.time()
    events = []  # (elapsed_s, label) goal-change markers for the graph

    for cycle in range(1, NUM_CYCLES + 1):
        print(f"\n=== Cycle {cycle}/{NUM_CYCLES} ===")

        events.append((round(time.time() - t0, 3), "-> MIN"))

        move_to_position_with_logging(SCS_ID, SCS_MIN_POSITION, SCS_MOVING_SPEED, SCS_MOVING_ACC,
                                       "moving_to_min", cycle, mass_g, DEFAULT_ABORT_CURRENT_MA,
                                       csv_writer, csv_file, samples, t0)
    
        hold_with_logging(SCS_ID, HOLD_SECONDS, "holding_min", SCS_MIN_POSITION, cycle, mass_g,
                           DEFAULT_ABORT_CURRENT_MA, csv_writer, csv_file, samples, t0)

        events.append((round(time.time() - t0, 3), "-> MAX"))

        move_to_position_with_logging(SCS_ID, SCS_MAX_POSITION, SCS_MOVING_SPEED, SCS_MOVING_ACC,
                                       "moving_to_max", cycle, mass_g, DEFAULT_ABORT_CURRENT_MA,
                                       csv_writer, csv_file, samples, t0)
        
        hold_with_logging(SCS_ID, HOLD_SECONDS, "holding_max", SCS_MAX_POSITION, cycle, mass_g,
                            DEFAULT_ABORT_CURRENT_MA, csv_writer, csv_file, samples, t0)

    events.append((round(time.time() - t0, 3), "-> CENTER"))

    move_to_position_with_logging(SCS_ID, SCS_CALIBRATED_CENTER, SCS_MOVING_SPEED, SCS_MOVING_ACC,
                                   "returning_to_center", NUM_CYCLES, mass_g, DEFAULT_ABORT_CURRENT_MA,
                                   csv_writer, csv_file, samples, t0)

    print("\nExperiment complete.")

except AbortExperiment as e:
    print(f"\n!!! ABORTED: {e} !!!")
except KeyboardInterrupt:
    print("\n!!! Interrupted by user (Ctrl+C) !!!")

#-----------------------------------------------------ENDING THE SCRIPT-------------------------------------------------------

finally:
    setSetting(address=SMS_STS_TORQUE_ENABLE, value=0, num_bytes=1, name="Torque Enable")
    print_summary(samples)
    if csv_file is not None:
        csv_file.close()
    if samples and csv_path is not None:
        plot_telemetry(samples, events, mass_g, csv_path)
    portHandler.closePort()
    print("Port Closed")
