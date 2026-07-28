#!/usr/bin/env python
"""
STS3215 EndoWrist Torque Characterization Test
================================================

Cycles a single EndoWrist drive disk up/down under a known hanging mass while
logging position/current/speed/load/voltage/temperature/error telemetry to a
CSV file, so the results can later be used to pick safe Torque Limit /
Overload Torque / Protection Torque / Overload Current / Protection Current /
Overcurrent Protection Time values for this instrument.

This script does NOT itself set any of those final EPROM protection values -
that is the whole point of the experiment (we don't know the right numbers
yet). It only applies a conservative *runtime* torque cap (SRAM Torque_Limit)
and slow speed/acceleration while collecting data.

SAFETY - read before running
-----------------------------
- Servo body AND EndoWrist shaft must both be rigidly clamped. Never hold the
  assembly by hand.
- Stop immediately (Ctrl+C) if: current rises unexpectedly, the cable frays,
  the instrument binds, the servo stalls, you hear unusual noises, or you see
  excessive vibration.
- Do not intentionally stall the servo for extended periods.
- This script also auto-aborts (disables torque) if present current exceeds
  --abort-current, or the servo reports an overheat/overload/overcurrent/
  overvoltage fault. Auto-abort is a backstop, not a replacement for watching
  the hardware.

Usage
-----
First run ever (or to redo the up/down limits):
    python torque_characterization_test.py --recalibrate --mass 0 --cycles 1

Normal run (one process per payload mass):
    python torque_characterization_test.py --mass 10 --cycles 20

Calibration is saved to endowrist_calibration.json next to this file and is
reused by every later run until --recalibrate is passed again.
"""

# --------------------------------------GENERAL IMPORTS----------------------------------------------------------

import argparse
import csv
import json
import os
import statistics
import sys
import time

print("GeneralImports Complete")

# ---------------------------------------SETTINGS BASED ON OS---------------------------------------------------
# Interactive jog-and-mark calibration needs raw-terminal arrow-key reads, same approach as
# prismatic_movement_test.py / quad_wheel_control.py in this folder. Those scripts are POSIX-only
# for the same reason, so this one is too.

if os.name == 'nt':
    raise RuntimeError("This script's interactive calibration requires Mac/Linux (uses select()/termios on stdin).")

import select
import termios
import tty

fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)

print("OS Based Settings Complete")

# ------------------------------------SERVO PYTHON LIBRARY IMPORT--------------------------------------------

sys.path.append("..")
from scservo_sdk import *  # Uses SC Servo SDK library
print("SC Servo Library Imported")

# -------------------------------------DEFAULT SERVO SETTINGS-------------------------------------------------

SCS_ID = 1                                                # SC Servo ID - EDIT for your rig
BAUDRATE = 1000000                                        # SC Servo default baudrate: 1000000 (1 Mbps)
DEVICENAME = '/dev/tty.usbmodem5AF71735721'               # EDIT: check which port is being used on your controller
                                                           # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

# Motion profile - deliberately slow per the experiment spec ("inertia should not dominate").
# Units are steps/sec (speed) and steps/sec^2 (acc), same convention as the other scripts here.
TEST_MOVING_SPEED = 300
TEST_MOVING_ACC = 10
HOLD_SECONDS = 2.0
DEFAULT_CYCLES = 20
MOVE_TIMEOUT_S = 10.0                                     # give up waiting for arrival after this long (still logs)

# Conservative runtime torque cap applied via SRAM Torque_Limit (address 48, see below) for the
# whole test, independent of and pending the final protection values this experiment will inform.
# 0-1000 = 0.0%-100.0% of max torque, same units as SCS_TORQUE_LIMIT in template.py.
TEST_TORQUE_LIMIT = 250                                   # 25.0% of max force

SAMPLE_HZ = 30
SAMPLE_PERIOD = 1.0 / SAMPLE_HZ

DEFAULT_ABORT_CURRENT_MA = 900                            # tune this once you have a sense of normal current draw

JOG_STEP = 8                                              # raw position units moved per arrow-key event during calibration
JOG_SPEED = 100
JOG_ACC = 10

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CALIBRATION_FILE = os.path.join(SCRIPT_DIR, "endowrist_calibration.json")
DATA_DIR = os.path.join(SCRIPT_DIR, "torque_test_data")

# ---------------------------------REGISTERS NOT DEFINED BY THIS SDK--------------------------------------------
# scservo_sdk/scservo_def.py already defines several register addresses (MIN_ANGLE_LIMIT_L=9,
# CW_DEAD=26, OFS_L=31, MODE=33, TORQUE_ENABLE=40, PRESENT_CURRENT_L=69, ...) that match the
# standard FeeTech STS/SMS memory table exactly, which gives good confidence in the rest of that
# same table for the two registers below (not exposed by this SDK):
STS_TORQUE_LIMIT_L = 48                                   # SRAM, 2 bytes, 0.1%/count - runtime torque cap
STS_STATUS = 65                                           # SRAM, 1 byte - lower confidence than the rest of this
                                                           # table; the per-packet error byte (scs_error, decoded via
                                                           # getRxPacketError) is the trusted fault signal in this
                                                           # script. Treat status_reg_raw in the CSV as a bonus
                                                           # diagnostic column, and cross-check against FeeTech's
                                                           # official memory table if it disagrees with observed
                                                           # behavior.

FAULT_ERRBITS = ERRBIT_VOLTAGE | ERRBIT_OVERHEAT | ERRBIT_OVERELE | ERRBIT_OVERLOAD


class AbortExperiment(Exception):
    """Raised to unwind cleanly to the safety shutdown path in main()."""


# --------------------------------------TERMINAL / KEY READING-----------------------------------------------

def getch():
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def read_key(timeout):
    """Non-blocking read of a single key event from stdin (raw fd, not buffered sys.stdin -
    see quad_wheel_control.py for why). Returns 'LEFT', 'RIGHT', 'ESC', a plain character, or None.
    """
    ready, _, _ = select.select([fd], [], [], timeout)
    if not ready:
        return None
    ch = os.read(fd, 1).decode()
    if ch != '\x1b':
        return ch
    ready, _, _ = select.select([fd], [], [], 0.1)
    if not ready:
        return 'ESC'
    ch2 = os.read(fd, 1).decode()
    if ch2 not in ('[', 'O'):
        return None
    ready, _, _ = select.select([fd], [], [], 0.1)
    if not ready:
        return None
    ch3 = os.read(fd, 1).decode()
    if ch3 == 'C':
        return 'RIGHT'
    elif ch3 == 'D':
        return 'LEFT'
    return None


# --------------------------------------SERVO COMMUNICATION--------------------------------------------------

def open_servo(devicename, baudrate):
    portHandler = PortHandler(devicename)
    packetHandler = sms_sts(portHandler)

    if not portHandler.openPort():
        raise RuntimeError(f"Failed to open port {devicename}")
    print("Succeeded to open the port")

    if not portHandler.setBaudRate(baudrate):
        portHandler.closePort()
        raise RuntimeError(f"Failed to set baudrate {baudrate}")
    print(f"Succeeded to change the baudrate to {baudrate}")

    return portHandler, packetHandler


def check(packetHandler, comm_result, error, context):
    if comm_result != COMM_SUCCESS:
        print(f"  [COMM WARN] {context}: {packetHandler.getTxRxResult(comm_result)}")
    elif error != 0:
        print(f"  [SERVO WARN] {context}: {packetHandler.getRxPacketError(error)}")


def set_position_mode(packetHandler, scs_id):
    # Guards against the servo being left in continuous-rotation Wheel Mode by quad_wheel_control.py.
    result, error = packetHandler.write1ByteTxRx(scs_id, SMS_STS_MODE, 0)
    check(packetHandler, result, error, "set position mode")


def set_torque_limit(packetHandler, scs_id, value):
    result, error = packetHandler.write2ByteTxRx(scs_id, STS_TORQUE_LIMIT_L, value)
    check(packetHandler, result, error, f"set torque limit to {value}")


def enable_torque(packetHandler, scs_id, enable):
    result, error = packetHandler.write1ByteTxRx(scs_id, SMS_STS_TORQUE_ENABLE, 1 if enable else 0)
    check(packetHandler, result, error, f"{'enable' if enable else 'disable'} torque")


def move_to(packetHandler, scs_id, position, speed, acc):
    result, error = packetHandler.WritePosEx(scs_id, position, speed, acc)
    check(packetHandler, result, error, f"move to {position}")


# --------------------------------------TELEMETRY ACQUISITION------------------------------------------------
# Present_Position(56) .. Present_Current(70) are contiguous in the memory table, so one raw read
# grabs everything needed for a sample in a single serial transaction - this is what makes reliable
# 20-50Hz sampling of every requested field possible over a shared serial bus.

def read_telemetry(packetHandler, scs_id):
    data, comm_result, error = packetHandler.readTxRx(scs_id, SMS_STS_PRESENT_POSITION_L, 15)
    if comm_result != COMM_SUCCESS or len(data) < 15:
        return None, comm_result, error

    def word(offset):
        return packetHandler.scs_makeword(data[offset], data[offset + 1])

    telemetry = {
        "present_position": packetHandler.scs_tohost(word(0), 15),
        "present_speed": packetHandler.scs_tohost(word(2), 15),
        "present_load_raw": word(4),
        "present_voltage": data[6],
        "present_temperature": data[7],
        "status_reg_raw": data[9],
        "present_current_ma": packetHandler.scs_tohost(word(13), 15),
    }
    return telemetry, comm_result, error


# --------------------------------------CALIBRATION (persisted, re-doable)------------------------------------

def load_calibration(scs_id):
    if not os.path.exists(CALIBRATION_FILE):
        return None
    with open(CALIBRATION_FILE, "r") as f:
        cal = json.load(f)
    if cal.get("scs_id") != scs_id:
        print(f"  [CALIBRATION] Saved calibration is for servo ID {cal.get('scs_id')}, not {scs_id} - recalibrating.")
        return None
    return cal


def save_calibration(scs_id, lower_position, upper_position):
    cal = {"scs_id": scs_id, "lower_position": lower_position, "upper_position": upper_position}
    with open(CALIBRATION_FILE, "w") as f:
        json.dump(cal, f, indent=2)
    print(f"Calibration saved to {CALIBRATION_FILE}: {cal}")
    return cal


def run_calibration(packetHandler, scs_id):
    """Interactive jog-and-mark: LEFT/RIGHT jog the servo, 'l' marks lower, 'u' marks upper,
    's' saves once both are marked, ESC aborts without saving.
    """
    present_position, comm_result, error = packetHandler.ReadPos(scs_id)
    check(packetHandler, comm_result, error, "read starting position")
    target = present_position

    lower_position = None
    upper_position = None

    print("\n--- Calibration: jog-and-mark ---")
    print("LEFT/RIGHT: jog   'l': mark LOWER here   'u': mark UPPER here   's': save & finish   ESC: abort")

    tty.setraw(fd)
    try:
        while True:
            key = read_key(0.05)
            if key == 'ESC':
                raise AbortExperiment("Calibration aborted by user, existing calibration (if any) left untouched.")
            elif key == 'LEFT':
                target -= JOG_STEP
                move_to(packetHandler, scs_id, target, JOG_SPEED, JOG_ACC)
            elif key == 'RIGHT':
                target += JOG_STEP
                move_to(packetHandler, scs_id, target, JOG_SPEED, JOG_ACC)
            elif key == 'l':
                present_position, comm_result, error = packetHandler.ReadPos(scs_id)
                check(packetHandler, comm_result, error, "read position for lower mark")
                lower_position = present_position
                print(f"\r\nMarked LOWER = {lower_position}")
            elif key == 'u':
                present_position, comm_result, error = packetHandler.ReadPos(scs_id)
                check(packetHandler, comm_result, error, "read position for upper mark")
                upper_position = present_position
                print(f"\r\nMarked UPPER = {upper_position}")
            elif key == 's':
                if lower_position is None or upper_position is None:
                    print("\r\nBoth LOWER and UPPER must be marked before saving.")
                    continue
                if lower_position == upper_position:
                    print("\r\nLOWER and UPPER cannot be the same position.")
                    continue
                break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    return save_calibration(scs_id, min(lower_position, upper_position), max(lower_position, upper_position))


def load_or_run_calibration(packetHandler, scs_id, force_recalibrate):
    if not force_recalibrate:
        cal = load_calibration(scs_id)
        if cal is not None:
            print(f"Using saved calibration: {cal}")
            return cal
    return run_calibration(packetHandler, scs_id)


# --------------------------------------DATA LOGGING----------------------------------------------------------

CSV_COLUMNS = [
    "timestamp_ms", "cycle", "phase", "goal_position", "present_position",
    "present_current_ma", "present_speed", "present_load_raw", "present_voltage",
    "present_temperature", "status_reg_raw", "packet_error_flags", "mass_g",
]


def open_csv(mass_g):
    os.makedirs(DATA_DIR, exist_ok=True)
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    path = os.path.join(DATA_DIR, f"torque_test_{mass_g}g_{timestamp}.csv")
    f = open(path, "w", newline="")
    writer = csv.writer(f)
    writer.writerow(CSV_COLUMNS)
    return f, writer, path


# --------------------------------------EXPERIMENT EXECUTION----------------------------------------------------

def sample_loop(packetHandler, scs_id, csv_writer, csv_file, samples, cycle, phase,
                 goal_position, mass_g, abort_current_ma, duration=None, until_stopped=False,
                 timeout=None):
    t_start = time.time()
    while True:
        sample_t0 = time.time()

        telemetry, comm_result, error = read_telemetry(packetHandler, scs_id)
        if telemetry is None:
            print(f"  [WARN] telemetry read failed ({packetHandler.getTxRxResult(comm_result)}), retrying...")
        else:
            row = {
                "timestamp_ms": int(sample_t0 * 1000),
                "cycle": cycle,
                "phase": phase,
                "goal_position": goal_position,
                "present_position": telemetry["present_position"],
                "present_current_ma": telemetry["present_current_ma"],
                "present_speed": telemetry["present_speed"],
                "present_load_raw": telemetry["present_load_raw"],
                "present_voltage": telemetry["present_voltage"],
                "present_temperature": telemetry["present_temperature"],
                "status_reg_raw": telemetry["status_reg_raw"],
                "packet_error_flags": error,
                "mass_g": mass_g,
            }
            csv_writer.writerow([row[col] for col in CSV_COLUMNS])
            csv_file.flush()
            samples.append(row)

            print(f"  cycle {cycle:2d} {phase:12s} goal={goal_position:5d} pos={telemetry['present_position']:5d} "
                  f"cur={telemetry['present_current_ma']:5d}mA spd={telemetry['present_speed']:5d} "
                  f"temp={telemetry['present_temperature']:3d}C volt={telemetry['present_voltage']:3d}")

            if telemetry["present_current_ma"] > abort_current_ma or telemetry["present_current_ma"] < -abort_current_ma:
                raise AbortExperiment(
                    f"present current {telemetry['present_current_ma']}mA exceeded abort threshold "
                    f"{abort_current_ma}mA during {phase} (cycle {cycle})")
            if error & FAULT_ERRBITS:
                raise AbortExperiment(
                    f"servo fault flags during {phase} (cycle {cycle}): {packetHandler.getRxPacketError(error)}")

        elapsed = time.time() - t_start
        if until_stopped:
            moving, comm_result, _ = packetHandler.ReadMoving(scs_id)
            if comm_result == COMM_SUCCESS and moving == 0:
                break
            if timeout is not None and elapsed > timeout:
                print(f"  [WARN] move timeout ({timeout}s) reached during {phase}, continuing anyway")
                break
        else:
            if elapsed >= duration:
                break

        dt = time.time() - sample_t0
        if dt < SAMPLE_PERIOD:
            time.sleep(SAMPLE_PERIOD - dt)


def print_summary(samples):
    if not samples:
        print("\nNo samples were collected.")
        return

    print("\n--- Summary ---")
    by_phase = {}
    for row in samples:
        by_phase.setdefault(row["phase"], []).append(row["present_current_ma"])

    for phase, currents in by_phase.items():
        print(f"  {phase:12s} mean current: {statistics.mean(currents):7.1f} mA   "
              f"peak current: {max(currents):5d} mA   n={len(currents)}")

    all_currents = [row["present_current_ma"] for row in samples]
    print(f"  {'overall':12s} max current observed: {max(all_currents)} mA   min: {min(all_currents)} mA")


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


# --------------------------------------MAIN--------------------------------------------------------------------

def parse_args():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mass", type=float, required=True, help="Hanging test mass in grams, logged into every row.")
    parser.add_argument("--cycles", type=int, default=DEFAULT_CYCLES, help=f"Number of up/down cycles (default {DEFAULT_CYCLES}).")
    parser.add_argument("--recalibrate", action="store_true", help="Force the jog-and-mark calibration flow even if a saved calibration exists.")
    parser.add_argument("--abort-current", type=float, default=DEFAULT_ABORT_CURRENT_MA,
                         help=f"Auto-abort if present current exceeds this many mA (default {DEFAULT_ABORT_CURRENT_MA}).")
    parser.add_argument("--port", type=str, default=DEVICENAME, help="Serial port device name.")
    parser.add_argument("--id", type=int, default=SCS_ID, help="Servo ID.")
    return parser.parse_args()


def main():
    args = parse_args()
    scs_id = args.id

    portHandler, packetHandler = open_servo(args.port, BAUDRATE)
    samples = []
    csv_file = None

    try:
        set_position_mode(packetHandler, scs_id)
        set_torque_limit(packetHandler, scs_id, TEST_TORQUE_LIMIT)
        enable_torque(packetHandler, scs_id, True)

        calibration = load_or_run_calibration(packetHandler, scs_id, args.recalibrate)
        lower_position = calibration["lower_position"]
        upper_position = calibration["upper_position"]

        csv_file, csv_writer, csv_path = open_csv(args.mass)
        print(f"Logging to {csv_path}")

        print_safety_reminder()

        for cycle in range(1, args.cycles + 1):
            print(f"\n=== Cycle {cycle}/{args.cycles} ===")

            move_to(packetHandler, scs_id, upper_position, TEST_MOVING_SPEED, TEST_MOVING_ACC)
            sample_loop(packetHandler, scs_id, csv_writer, csv_file, samples, cycle, "moving_up",
                        upper_position, args.mass, args.abort_current, until_stopped=True, timeout=MOVE_TIMEOUT_S)
            sample_loop(packetHandler, scs_id, csv_writer, csv_file, samples, cycle, "holding_up",
                        upper_position, args.mass, args.abort_current, duration=HOLD_SECONDS)

            move_to(packetHandler, scs_id, lower_position, TEST_MOVING_SPEED, TEST_MOVING_ACC)
            sample_loop(packetHandler, scs_id, csv_writer, csv_file, samples, cycle, "moving_down",
                        lower_position, args.mass, args.abort_current, until_stopped=True, timeout=MOVE_TIMEOUT_S)
            sample_loop(packetHandler, scs_id, csv_writer, csv_file, samples, cycle, "holding_down",
                        lower_position, args.mass, args.abort_current, duration=HOLD_SECONDS)

        print("\nExperiment complete.")

    except AbortExperiment as e:
        print(f"\n!!! ABORTED: {e} !!!")
    except KeyboardInterrupt:
        print("\n!!! Interrupted by user (Ctrl+C) !!!")
    finally:
        enable_torque(packetHandler, scs_id, False)
        print_summary(samples)
        if csv_file is not None:
            csv_file.close()
        portHandler.closePort()
        print("Port Closed")


if __name__ == "__main__":
    main()
