#!/usr/bin/env python
"""Teleoperate all 10 follower-arm motors at once: the base arm (motors 1-5) mirrors
the leader arm, while the wrist (motor 6 = prismatic slide, motors 7-10 = rotary discs)
is jogged by held keyboard keys.

Motors 1-5 and 6-10 are daisy-chained on ONE shared physical serial port, so this script
deliberately does NOT use two separate bus objects (lerobot's FeetechMotorsBus for 1-5 and
the repo's own hand-rolled sms_sts.Bus for 6-10) -- opening the same port twice would
corrupt communication. Instead, ONE FeetechMotorsBus is built covering all 10 motors:
motors 1-5 keep their real lerobot calibration (loaded from the existing
so101_follower_5dof calibration file); motors 6-10 have no calibration entry and are
always accessed with normalize=False (raw encoder-tick space), reusing calibration.py's
exact sensorless-homing sequence and protection/alarm/dead-zone settings, ported onto
this bus's read/write API.

Robot.connect()/calibrate()/configure()/send_action()/get_observation() are intentionally
NOT used on this 10-motor bus: is_calibrated() would permanently mismatch (only 5 of the
10 motors have a calibration entry) and force an interactive calibration prompt on every
launch. The lower-level bus.connect()/read()/write()/sync_read()/sync_write() are used
directly instead.

Key bindings (first key = positive direction, second = negative):
    Left/Right arrow -> motor 6 (prismatic slide).      Left = positive, Right = negative.
    Q / E             -> motor 7 (rotary disc).
    I / P             -> motor 8 (rotary disc).
    Z / C             -> motor 9 (rotary disc).
    B / M             -> motor 10 (rotary disc).
ESC or Ctrl+C exits. Motor 6 is re-homed (sensorless, drives into both mechanical stops)
every time this script starts. After homing completes, ALL 10 motors -- base arm and
wrist alike -- stay locked out from any control for 5 seconds before teleop goes live.

Usage:
    07_teleoperate_full_10dof.py \
        --leader-port /dev/ttyACM0 --leader-id my_leader_arm \
        --follower-port /dev/ttyACM1 --follower-id my_follower_arm
"""

import argparse
import os
import sys
from pathlib import Path

if os.name == "nt":
    raise RuntimeError(
        "This combined teleop script requires Mac/Linux (uses select()/termios on stdin)."
    )

import select
import termios
import time
import tty

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from lerobot.motors import Motor, MotorNormMode  # noqa: E402
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode  # noqa: E402
from lerobot.robots.so_follower import SOFollowerRobotConfig  # noqa: E402
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig  # noqa: E402

from so101_follower_5dof import SO101Follower5DOF  # noqa: E402

# --------------------------------------------------------------------------------------
# Constants
# --------------------------------------------------------------------------------------

ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
WRIST_MOTOR_NAMES = {6: "prismatic", 7: "rotary_7", 8: "rotary_8", 9: "rotary_9", 10: "rotary_10"}

# --- Motor 6 (prismatic, sensorless homing) -- values ported 1:1 from calibration.py ---
PRISMATIC_ID = 6
HOMING_SPEED, HOMING_ACC = 300, 30
MOVING_SPEED_M6, MOVING_ACC_M6 = 1000, 100
RANGE_BUFFER = 1000
BACKOFF = 2048
SCS_MOVE_TOLERANCE = 3
SEARCH_DISTANCE = 20000
SAMPLE_HZ = 30
SAMPLE_PERIOD = 1.0 / SAMPLE_HZ
SPIKE_CURRENT_THRESHOLD_MA = 35
STOP_SETTLE_TIMEOUT_S = 4.0
SETTLE_DWELL_S = 0.3
HOMING_TORQUE_LIMIT = 125          # "Torque_Limit" register (addr 48), 0.1% units --
MOVING_TORQUE_LIMIT_M6 = 400       # NOT "Max_Torque_Limit" (addr 16, a different EEPROM register)

# --- Motors 7-10 (rotary, calibrated center) ---
SCS_CALIBRATED_CENTER = 2048
ROTARY_TORQUE_LIMIT = 125
MOTOR_CONFIGS = {
    7:  {"range": 700,  "moving_speed": 1200, "moving_acc": 10, "margin": 100},
    8:  {"range": 700,  "moving_speed": 1200, "moving_acc": 10, "margin": 100},
    9:  {"range": 1400, "moving_speed": 1200, "moving_acc": 10, "margin": 100},
    10: {"range": 1400, "moving_speed": 1200, "moving_acc": 10, "margin": 100},
}

# --- Protection/alarm/dead-zone, applied identically to all of 6-10 ---
MAX_TEMPERATURE_LIMIT = 55
MAX_VOLTAGE_LIMIT, MIN_VOLTAGE_LIMIT = 130, 90
UNLOADING_CONDITIONS, LED_ALARM_CONDITIONS = 63, 63
CW_INSENSITIVE_ZONE, CCW_INSENSITIVE_ZONE = 1, 1

POST_HOMING_LOCKOUT_S = 5.0        # gates ALL 10 motors, base arm included

# --- Keyboard jog control ---
RELEASE_TIMEOUT = 0.15
POLL_INTERVAL = 0.02               # also serves as the combined loop's tick pacer
ESCAPE_SEQUENCE_TIMEOUT = 0.1

# Left=positive / Right=negative, Q/E, I/P, Z/C, B/M
KEY_BINDINGS = {
    "LEFT": (6, +1), "RIGHT": (6, -1),
    "q": (7, +1),  "e": (7, -1),
    "i": (8, +1),  "p": (8, -1),
    "z": (9, +1),  "c": (9, -1),
    "b": (10, +1), "m": (10, -1),
}

# Raw encoder counts added to (or subtracted from) a wrist motor's goal position on every
# loop tick while its key is held -- this is what sets jog speed. At POLL_INTERVAL=0.02s,
# a step of 15 means +15 counts every 20ms =~ 750 counts/sec =~ 66 deg/sec for a full
# 4096-count revolution. No prior art to base these on -- first-guess placeholders, meant
# to be tuned live once the hardware is running.
WRIST_STEP_PER_TICK = {6: 15, 7: 8, 8: 8, 9: 15, 10: 15}

fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)


class HomingError(Exception):
    """Raised when a search leg fails to find an obstacle within SEARCH_DISTANCE, or an
    unexpected current spike occurs while driving to the computed center."""


# --------------------------------------------------------------------------------------
# Keyboard input (POSIX raw-mode, non-blocking poll -- ported from quad_wheel_control.py)
# --------------------------------------------------------------------------------------


def read_key(timeout):
    """Non-blocking read of a single key event from stdin.
    Returns 'LEFT', 'RIGHT', 'ESC', a plain character (e.g. 'q', 'B'), or None if nothing
    usable arrived within timeout.

    Reads directly from the raw file descriptor (os.read) rather than the buffered
    sys.stdin: a buffered read can silently slurp a whole arrow-key escape sequence into
    its internal buffer on the first call, leaving nothing at the OS level for the next
    select() to see - making a real arrow press look like a lone ESC.
    """
    ready, _, _ = select.select([fd], [], [], timeout)
    if not ready:
        return None
    ch = os.read(fd, 1).decode()
    if ch != "\x1b":
        return ch
    ready, _, _ = select.select([fd], [], [], ESCAPE_SEQUENCE_TIMEOUT)
    if not ready:
        return "ESC"
    ch2 = os.read(fd, 1).decode()
    if ch2 not in ("[", "O"):
        return None
    ready, _, _ = select.select([fd], [], [], ESCAPE_SEQUENCE_TIMEOUT)
    if not ready:
        return None
    ch3 = os.read(fd, 1).decode()
    if ch3 == "C":
        return "RIGHT"
    elif ch3 == "D":
        return "LEFT"
    return None


# --------------------------------------------------------------------------------------
# Motor 6 sensorless homing -- ported from calibration.py onto bus.read/write
# --------------------------------------------------------------------------------------


CURRENT_MA_PER_COUNT = 6.5  # Present_Current register units -- matches motor.py's getTelemetry()


def read_prismatic_telemetry(bus):
    position = bus.read("Present_Position", "prismatic", normalize=False)
    raw_current = bus.read("Present_Current", "prismatic", normalize=False)
    current_ma = raw_current * CURRENT_MA_PER_COUNT
    moving = bus.read("Moving", "prismatic", normalize=False)
    return position, current_ma, moving


def wait_for_stop(bus):
    """Polls until the motor reports moving==0 (the stop command has taken effect), then
    dwells SETTLE_DWELL_S longer so residual current decays. Returns the settled position."""
    position = None

    settle_deadline = time.time() + STOP_SETTLE_TIMEOUT_S
    while time.time() < settle_deadline:
        sample_t0 = time.time()
        position, _, moving = read_prismatic_telemetry(bus)
        if moving == 0:
            break
        dt = time.time() - sample_t0
        if dt < SAMPLE_PERIOD:
            time.sleep(SAMPLE_PERIOD - dt)
    else:
        print(
            f"  [WARN] motor still reports moving after {STOP_SETTLE_TIMEOUT_S}s settle "
            "timeout - proceeding anyway"
        )

    dwell_deadline = time.time() + SETTLE_DWELL_S
    while time.time() < dwell_deadline:
        sample_t0 = time.time()
        position, _, _ = read_prismatic_telemetry(bus)
        dt = time.time() - sample_t0
        if dt < SAMPLE_PERIOD:
            time.sleep(SAMPLE_PERIOD - dt)

    return position


def search_for_obstacle(bus, direction_label, search_goal_position, speed, acceleration):
    """Blocking move toward search_goal_position. The instant present current reaches
    SPIKE_CURRENT_THRESHOLD_MA, stops, waits to settle, backs off BACKOFF steps, and
    returns the settled obstacle position. Raises HomingError if the search goal is
    reached without ever seeing a spike."""
    bus.write("Goal_Velocity", "prismatic", speed, normalize=False)
    bus.write("Acceleration", "prismatic", acceleration, normalize=False)
    bus.write("Goal_Position", "prismatic", search_goal_position, normalize=False)

    has_moved = False
    while True:
        sample_t0 = time.time()
        position, current_ma, moving = read_prismatic_telemetry(bus)

        print(
            f"  {direction_label:10s} GoalPos:{search_goal_position:6d} PresPos:{position:6d} "
            f"Current:{current_ma:7.1f}mA"
        )

        if abs(current_ma) >= SPIKE_CURRENT_THRESHOLD_MA:
            bus.write("Goal_Position", "prismatic", position, normalize=False)
            print(
                f"  *** Obstacle detected {direction_label} near position {position} "
                f"(current {current_ma:.1f}mA >= {SPIKE_CURRENT_THRESHOLD_MA}mA) - settling ***"
            )
            obstacle_position = wait_for_stop(bus)
            if obstacle_position is None:
                obstacle_position = position
            print(f"  *** Settled at {obstacle_position} ***")

            backoff_direction = 1 if search_goal_position > obstacle_position else -1
            backoff_goal = obstacle_position - backoff_direction * BACKOFF
            backoff_label = "backward" if backoff_direction > 0 else "forward"
            print(f"  *** Backing off {BACKOFF} steps {backoff_label} to {backoff_goal} ***")
            bus.write("Goal_Position", "prismatic", backoff_goal, normalize=False)
            wait_for_stop(bus)

            return obstacle_position

        if moving == 1:
            has_moved = True
        if moving == 0 and (has_moved or abs(position - search_goal_position) <= SCS_MOVE_TOLERANCE):
            raise HomingError(
                f"No obstacle detected {direction_label}: reached search goal "
                f"{search_goal_position} without current ever reaching "
                f"{SPIKE_CURRENT_THRESHOLD_MA}mA."
            )

        dt = time.time() - sample_t0
        if dt < SAMPLE_PERIOD:
            time.sleep(SAMPLE_PERIOD - dt)


def drive_to_position(bus, goal_position, phase_label, speed, acceleration):
    """Blocking move to goal_position. Aborts with HomingError if current unexpectedly
    spikes mid-move."""
    bus.write("Goal_Velocity", "prismatic", speed, normalize=False)
    bus.write("Acceleration", "prismatic", acceleration, normalize=False)
    bus.write("Goal_Position", "prismatic", goal_position, normalize=False)

    has_moved = False
    while True:
        sample_t0 = time.time()
        position, current_ma, moving = read_prismatic_telemetry(bus)

        print(
            f"  {phase_label:14s} GoalPos:{goal_position:6d} PresPos:{position:6d} "
            f"Current:{current_ma:7.1f}mA"
        )

        if abs(current_ma) >= SPIKE_CURRENT_THRESHOLD_MA:
            bus.write("Goal_Position", "prismatic", position, normalize=False)
            raise HomingError(
                f"Unexpected obstacle during '{phase_label}' at position {position} "
                f"(current {current_ma:.1f}mA >= {SPIKE_CURRENT_THRESHOLD_MA}mA)."
            )

        if moving == 1:
            has_moved = True
        if moving == 0 and (has_moved or abs(position - goal_position) <= SCS_MOVE_TOLERANCE):
            break

        dt = time.time() - sample_t0
        if dt < SAMPLE_PERIOD:
            time.sleep(SAMPLE_PERIOD - dt)


def home_motor_6(bus):
    """Sensorless homing for the prismatic joint: drives into both mechanical stops
    (detected via a current spike), computes the center, and returns (min, max, goal)
    dicts (keyed by motor id) with motor 6's discovered software bounds."""

    print("\n=== Step 1: Multi-turn Phase bit set (Angle Resolution stays 1) ===")
    with bus.torque_disabled(["prismatic"]):
        bus.write("Angular_Resolution", "prismatic", 1, normalize=False)
        phase = bus.read("Phase", "prismatic", normalize=False)
        bus.write("Phase", "prismatic", phase | 0x10, normalize=False)

    print("\n=== Step 2: Min/Max Angle Limit -> unrestricted (multi-turn) ===")
    with bus.torque_disabled(["prismatic"]):
        bus.write("Min_Position_Limit", "prismatic", 0, normalize=False)
        bus.write("Max_Position_Limit", "prismatic", 0, normalize=False)

    print("\n=== Step 3: Reset Position Correction to 0 ===")
    with bus.torque_disabled(["prismatic"]):
        bus.write("Homing_Offset", "prismatic", 0, normalize=False)

    print("\n=== Step 4: Cap runtime torque for homing ===")
    bus.write("Torque_Limit", "prismatic", HOMING_TORQUE_LIMIT, normalize=False)

    print("\n=== Step 5: Enable torque ===")
    bus.write("Torque_Enable", "prismatic", 1, normalize=False)
    time.sleep(1)

    start_position = bus.read("Present_Position", "prismatic", normalize=False)
    print(f"\nStart position (raw, unwrapped): {start_position}")

    print("\n=== Step 6: Search forward for obstacle #1 ===")
    forward_position = search_for_obstacle(
        bus, "forward", start_position + SEARCH_DISTANCE, HOMING_SPEED, HOMING_ACC
    )

    print("\n=== Step 7: Search backward for obstacle #2 ===")
    back_position = search_for_obstacle(
        bus, "backward", forward_position - SEARCH_DISTANCE, HOMING_SPEED, HOMING_ACC
    )

    center_position = round((back_position + forward_position) / 2)
    print(f"\nBack obstacle (unwrapped):    {back_position}")
    print(f"Forward obstacle (unwrapped): {forward_position}")
    print(f"Computed center (unwrapped):  {center_position}")

    print("\n=== Step 8: Drive to computed center ===")
    drive_to_position(bus, center_position, "moving_to_center", HOMING_SPEED, HOMING_ACC)
    time.sleep(2)

    print("\n=== Step 9: Raise motor 6 runtime torque for normal operation ===")
    bus.write("Torque_Limit", "prismatic", MOVING_TORQUE_LIMIT_M6, normalize=False)
    bus.write("Goal_Velocity", "prismatic", MOVING_SPEED_M6, normalize=False)
    bus.write("Acceleration", "prismatic", MOVING_ACC_M6, normalize=False)

    wrist_min = {PRISMATIC_ID: back_position + RANGE_BUFFER}
    wrist_max = {PRISMATIC_ID: forward_position - RANGE_BUFFER}
    wrist_goal = {PRISMATIC_ID: center_position}

    print(f"Motor {PRISMATIC_ID} range-test min (buffered): {wrist_min[PRISMATIC_ID]}")
    print(f"Motor {PRISMATIC_ID} range-test max (buffered): {wrist_max[PRISMATIC_ID]}")

    return wrist_min, wrist_max, wrist_goal


# --------------------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--leader-port", required=True)
    parser.add_argument("--leader-id", required=True)
    parser.add_argument("--follower-port", required=True)
    parser.add_argument("--follower-id", required=True)
    args = parser.parse_args()

    # 1. Load motors 1-5's existing calibration without ever opening the port.
    calib_source = SO101Follower5DOF(
        SOFollowerRobotConfig(port=args.follower_port, id=args.follower_id)
    )
    calib_1_5 = calib_source.calibration
    if not calib_1_5:
        raise RuntimeError(
            f"No calibration found for follower id '{args.follower_id}' -- run "
            "05_calibrate_follower.py first."
        )
    config = calib_source.config
    del calib_source  # never call .connect() on it

    # 2. Build ONE bus owning all 10 motors.
    norm_mode_body = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100
    motors = {name: Motor(i, "sts3215", norm_mode_body) for i, name in zip(range(1, 6), ARM_JOINTS)}
    for wid, wname in WRIST_MOTOR_NAMES.items():
        # Placeholder norm_mode -- unused, every wrist read/write always passes normalize=False.
        motors[wname] = Motor(wid, "sts3215", MotorNormMode.RANGE_M100_100)
    bus = FeetechMotorsBus(port=args.follower_port, motors=motors, calibration=calib_1_5)

    leader = SO101Leader(SO101LeaderConfig(port=args.leader_port, id=args.leader_id))

    try:
        # 3. Connect leader (separate port) and bus (shared port), both low-level.
        leader.connect()
        bus.connect()  # handshake: pings all 10 IDs + firmware check only

        # 4. Push known-good calibration onto 1-5's EEPROM.
        bus.write_calibration(calib_1_5, cache=True)

        # 5. configure_motors() BEFORE motor 6's homing -- it clears the sts3215 Phase bit
        #    0x10 on every motor, which would undo motor 6's multi-turn phase bit if run after.
        bus.configure_motors()

        # 6. Base-arm motor config (mirrors SOFollower.configure(), scoped to motors 1-5).
        with bus.torque_disabled(list(ARM_JOINTS)):
            for name in ARM_JOINTS:
                bus.write("Operating_Mode", name, OperatingMode.POSITION.value)
                bus.write("P_Coefficient", name, config.position_p_coefficient)
                bus.write("I_Coefficient", name, config.position_i_coefficient)
                bus.write("D_Coefficient", name, config.position_d_coefficient)

        # 7. Protection/alarm/dead-zone settings on ALL of 6-10, BEFORE motor 6 homing.
        with bus.torque_disabled(list(WRIST_MOTOR_NAMES.values())):
            for wname in WRIST_MOTOR_NAMES.values():
                bus.write("Max_Temperature_Limit", wname, MAX_TEMPERATURE_LIMIT, normalize=False)
                bus.write("Max_Voltage_Limit", wname, MAX_VOLTAGE_LIMIT, normalize=False)
                bus.write("Min_Voltage_Limit", wname, MIN_VOLTAGE_LIMIT, normalize=False)
                bus.write("Unloading_Condition", wname, UNLOADING_CONDITIONS, normalize=False)
                bus.write("LED_Alarm_Condition", wname, LED_ALARM_CONDITIONS, normalize=False)
                bus.write("CW_Dead_Zone", wname, CW_INSENSITIVE_ZONE, normalize=False)
                bus.write("CCW_Dead_Zone", wname, CCW_INSENSITIVE_ZONE, normalize=False)

        # 8. Motors 7-10 hardware backup position limits + explicit torque enable.
        with bus.torque_disabled([WRIST_MOTOR_NAMES[i] for i in MOTOR_CONFIGS]):
            for wid, cfg in MOTOR_CONFIGS.items():
                wname = WRIST_MOTOR_NAMES[wid]
                min_pos = SCS_CALIBRATED_CENTER - cfg["range"]
                max_pos = SCS_CALIBRATED_CENTER + cfg["range"]
                bus.write("Min_Position_Limit", wname, min_pos - cfg["margin"], normalize=False)
                bus.write("Max_Position_Limit", wname, max_pos + cfg["margin"], normalize=False)
                bus.write("Operating_Mode", wname, OperatingMode.POSITION.value, normalize=False)
                bus.write("Goal_Velocity", wname, cfg["moving_speed"], normalize=False)
                bus.write("Acceleration", wname, cfg["moving_acc"], normalize=False)
                bus.write("Torque_Limit", wname, ROTARY_TORQUE_LIMIT, normalize=False)
        for wid in MOTOR_CONFIGS:
            bus.write("Torque_Enable", WRIST_MOTOR_NAMES[wid], 1, normalize=False)

        # 9. Motor 6 sensorless homing.
        try:
            wrist_min, wrist_max, wrist_goal = home_motor_6(bus)
        except HomingError as e:
            print(f"\n!!! HOMING FAILED: {e} !!!")
            bus.write("Torque_Enable", "prismatic", 0, normalize=False)
            return

        for wid, cfg in MOTOR_CONFIGS.items():
            wrist_min[wid] = SCS_CALIBRATED_CENTER - cfg["range"]
            wrist_max[wid] = SCS_CALIBRATED_CENTER + cfg["range"]
            # Seed from truth (not an assumed constant) to avoid a jump on the first tick.
            wrist_goal[wid] = bus.read("Present_Position", WRIST_MOTOR_NAMES[wid], normalize=False)

        # 10. 5-second lockout gating BOTH forms of teleop -- all 10 motors hold their
        #     current commanded position (torque stays enabled, no writes sent).
        print(f"\nHoming complete. Starting teleop in {POST_HOMING_LOCKOUT_S:.0f}s...")
        time.sleep(POST_HOMING_LOCKOUT_S)
        termios.tcflush(fd, termios.TCIFLUSH)  # drop keys buffered by the OS during the wait

        # 11. Combined keyboard + leader-follower teleop loop.
        print(
            "Teleoperating. Hold Left/Right for motor 6, Q/E for 7, I/P for 8, Z/C for 9, "
            "B/M for 10. Move the leader arm for motors 1-5. ESC or Ctrl+C to quit."
        )
        tty.setraw(fd)
        try:
            last_key_time = {wid: 0.0 for wid in WRIST_MOTOR_NAMES}
            current_sign = {wid: 0 for wid in WRIST_MOTOR_NAMES}

            while True:
                key = read_key(POLL_INTERVAL)
                now = time.time()

                if key == "ESC" or key == "\x03":
                    # Raw mode disables ISIG, so Ctrl+C arrives as byte '\x03', not a
                    # real KeyboardInterrupt -- must be checked explicitly.
                    break

                k = key if key in ("LEFT", "RIGHT") else (key.lower() if key else None)
                if k in KEY_BINDINGS:
                    wid, sign = KEY_BINDINGS[k]
                    last_key_time[wid] = now
                    current_sign[wid] = sign

                wrist_changed = {}
                for wid in WRIST_MOTOR_NAMES:
                    if current_sign[wid] != 0 and (now - last_key_time[wid]) > RELEASE_TIMEOUT:
                        current_sign[wid] = 0  # released: goal simply stops changing
                    if current_sign[wid] != 0:
                        new_goal = wrist_goal[wid] + current_sign[wid] * WRIST_STEP_PER_TICK[wid]
                        wrist_goal[wid] = max(wrist_min[wid], min(wrist_max[wid], new_goal))
                        wrist_changed[WRIST_MOTOR_NAMES[wid]] = wrist_goal[wid]

                if wrist_changed:
                    bus.sync_write("Goal_Position", wrist_changed, normalize=False)

                raw_action = leader.get_action()
                arm_action = {name: raw_action[f"{name}.pos"] for name in ARM_JOINTS}
                bus.sync_write("Goal_Position", arm_action)  # normalize=True (default)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    except KeyboardInterrupt:
        print("\n!!! Interrupted by user (Ctrl+C) !!!")
    finally:
        if bus.is_connected:
            bus.disconnect()  # default disable_torque=True -- disables all 10 motors, closes the port
        if leader.is_connected:
            leader.disconnect()


if __name__ == "__main__":
    main()
