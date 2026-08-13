# -----------------------------------------------------GENERAL IMPORTS----------------------------------------------------------

print("Importing time")
import time

print("Importing bus class")
from bus import Bus

print("Importing Telemetry class")
from telemetry import Telemetry

print("Importing SC Servo SDK")
from scservo_sdk import *  # Uses SC Servo SDK library

print("General Imports Complete\n")

# -------------------------------------------------------INITIALIZE SERVO SETTINGS-------------------------------------------------

BAUDRATE = 1000000  # SC Servo default baudrate : 1000000 (1 Mbps)
DEVICENAME = "/dev/ttyACM0"  # Check which port is being used on your controller

# --- Motor 6 (prismatic joint, sensorless homing) ---

PRISMATIC_ID = 6  # CHANGE THIS to the ID of the prismatic motor to home

homingSpeed = 300  # Matches the run that produced the current-draw baseline (prismaticCurrentDraw.py). Used for obstacle search + initial drive-to-center.
homingAcc = 30
movingSpeed = (
    1000  # Used only for motor 6's range-test moves (front-most/back-most/center)
)
movingAcc = 100
RANGE_BUFFER = 1000  # Steps the range-test bounds sit inside each detected obstacle, so movingSpeed/movingAcc doesn't ram the stop
BACKOFF = 2048  # Steps (half a rev) to retreat from an obstacle right after detecting it, so current returns to baseline before the next move
SCS_MOVE_TOLERANCE = 3  # Steps within which we consider a move "arrived"

# This joint is driven by a lead screw, so it needs several full servo revolutions to cover
# its prismatic travel. SEARCH_DISTANCE just has to be comfortably larger than the true
# travel so each search leg is guaranteed to hit its mechanical stop before running out of
# commanded distance - it does NOT need to be a tight bound (searchForObstacle() below stops
# the instant a current spike is seen, long before actually reaching this goal).
SEARCH_DISTANCE = 20000

SAMPLE_HZ = 30  # Telemetry sampling rate during motion
SAMPLE_PERIOD = 1.0 / SAMPLE_HZ

# From prismaticCurrentDraw.py's free-travel current characterization at this same
# speed/acc (SCS_ID 6, angle resolution 1): baseline mean 11.4mA, stdev 3.9mA, peak 19.5mA
# across forward+backward legs -> mean + 4*stdev ~= 27mA. Comfortably above the observed
# free-travel peak, well below what a real stall against the mechanical stop produces.
SPIKE_CURRENT_THRESHOLD_MA = 35

# After a spike stops the goal-position command, the joint keeps coasting/decelerating for a
# moment before it's actually at rest - starting the next leg's motion during that window
# reverses direction while still moving, which itself draws a current spike from braking, not
# from a real second obstacle. waitForStop() below polls until moving==0 (or this timeout),
# then dwells an extra beat so residual current has time to decay before the next leg begins.
STOP_SETTLE_TIMEOUT_S = 4.0
SETTLE_DWELL_S = 0.3

# SRAM, 2 bytes, 0.1%/count - runtime torque cap (address 48). Not exposed by name in this
# SDK, but matches the standard FeeTech STS/SMS memory table (same table that gives
# MIN_ANGLE_LIMIT_L=9, OFS_L=31, MODE=33, TORQUE_ENABLE=40, PRESENT_CURRENT_L=69, ...
# all already defined by this SDK). Shared across every motor on the bus - same register
# regardless of motor ID.
STS_TORQUE_LIMIT_L = 48
HOMING_TORQUE_LIMIT = (
    125  # 12.5% of max torque - keeps stop impacts gentle during search.
)
movingTorqueLimit = 400  # 40% of max torque - raised on motor 6 only, after homing, for its own range test.

# --- Motors 7-10 (rotary joints, already offset-calibrated via positionCalibrationOOP.py) ---

SCS_CALIBRATED_CENTER = 2048  # The position value that is each rotary motor's true center after offset correction

ROTARY_TORQUE_LIMIT = 125  # 12.5% of max torque, applied to motors 7-10 before their range test (they have no torque limit set otherwise)

# Per-motor configuration. "margin" separates the software position limits (minPosition/
# maxPosition) from the hardware backup limits (minPositionLimit/maxPositionLimit): the
# hardware limit is set `margin` steps wider on each side, so it only kicks in if the
# software limit fails.
MOTOR_CONFIGS = {
    7: {
        "range": 700,
        "moving_speed": 1200,
        "moving_acc": 10,
        "tolerance": 3,
        "margin": 100,
    },
    8: {
        "range": 700,
        "moving_speed": 1200,
        "moving_acc": 10,
        "tolerance": 3,
        "margin": 100,
    },
    9: {
        "range": 1400,
        "moving_speed": 1200,
        "moving_acc": 10,
        "tolerance": 3,
        "margin": 100,
    },
    10: {
        "range": 1400,
        "moving_speed": 1200,
        "moving_acc": 10,
        "tolerance": 3,
        "margin": 100,
    },
}

print(f"SC Servo BAUDRATE: {BAUDRATE}")
print(f"SC Servo DEVICENAME: {DEVICENAME}")
print(f"Prismatic Motor ID: {PRISMATIC_ID}")
print(f"Homing Speed: {homingSpeed}")
print(f"Homing Acc: {homingAcc}")
print(f"Moving Speed (motor {PRISMATIC_ID}): {movingSpeed}")
print(f"Moving Acc (motor {PRISMATIC_ID}): {movingAcc}")
print(f"Range Test Buffer: {RANGE_BUFFER}")
print(f"Search Distance: {SEARCH_DISTANCE}")
print(f"Spike Current Threshold (mA): {SPIKE_CURRENT_THRESHOLD_MA}")
print(f"Homing Torque Limit: {HOMING_TORQUE_LIMIT / 10.0:.1f}%")
print(f"SC Servo Calibrated Center (motors 7-10): {SCS_CALIBRATED_CENTER}")
print(f"Rotary Torque Limit (motors 7-10): {ROTARY_TORQUE_LIMIT / 10.0:.1f}%")
print("")

# --------------------------------------------------BUS AND MOTOR INITIALIZATION-------------------------------------------------

bus = Bus(DEVICENAME, BAUDRATE)


class HomingError(Exception):
    """Raised when a search leg fails to find an obstacle within SEARCH_DISTANCE, or an
    unexpected current spike occurs while driving to the computed center."""


with bus:

    # Build one Motor per joint, in order 6, 7, 8, 9, 10. motor6 gets an unrestricted
    # multi-turn range (its true travel is only known after homing, below); motors 7-10
    # are constructed the same way rangeTestOOP.py builds them, from MOTOR_CONFIGS.
    motors = {}

    motors[PRISMATIC_ID] = bus.addMotor(
        id=PRISMATIC_ID,
        rawCenterPosition=0,
        range=SEARCH_DISTANCE,
        minPosition=-SEARCH_DISTANCE,
        maxPosition=SEARCH_DISTANCE,
        positionTolerance=SCS_MOVE_TOLERANCE,
        minPositionLimit=0,
        maxPositionLimit=0,  # 0/0 = unrestricted multi-turn rotation (no clipping at +/-4095)
        positionMargin=0,
        angleResolution=1,
    )
    motor6 = motors[PRISMATIC_ID]

    for id, cfg in MOTOR_CONFIGS.items():
        minPosition = SCS_CALIBRATED_CENTER - cfg["range"]
        maxPosition = SCS_CALIBRATED_CENTER + cfg["range"]
        margin = cfg["margin"]
        motors[id] = bus.addMotor(
            id=id,
            rawCenterPosition=SCS_CALIBRATED_CENTER,
            range=cfg["range"],
            minPosition=minPosition,
            maxPosition=maxPosition,
            positionTolerance=cfg["tolerance"],
            minPositionLimit=minPosition - margin,
            maxPositionLimit=maxPosition + margin,
            positionMargin=margin,
            angleResolution=cfg.get("angleResolution", 1),
        )

    bus.printAllSettings()
    for motor in motors.values():
        motor.printAllSettings()

    # Write the margin-derived hardware backup limits (minPositionLimit/maxPositionLimit) to
    # motors 7-10's EEPROM before running the range check. Motor 6's position limits are
    # applied as part of its own homing sequence below (Step 2), since it needs a specific
    # unlock/lock order alongside the rest of that setup.
    print("--- Applying Hardware Position Limits (motors 7-10) ---")
    for id in MOTOR_CONFIGS:
        motors[id].applyPositionLimits()

    telemetryPlot = None
    backPosition = None
    forwardPosition = None

    try:
        print("\n=== Step 1: Multi-turn Phase bit set (Angle Resolution stays 1) ===")
        motor6.applyAngleResolution()  # unlocks EEPROM, re-asserts Angle Resolution=1, sets Phase BIT4, re-locks

        print("\n=== Step 2: Min/Max Angle Limit -> unrestricted (multi-turn) ===")
        motor6.applyPositionLimits()

        print("\n=== Step 3: Reset Position Correction to 0 ===")
        motor6.unlockEeprom()
        # Position Correction (address 31) is only ever reset to 0 here, so the default bit-15
        # signed encoding in setSetting is fine - 0 is unaffected by which bit is treated as sign.
        motor6.setSetting(
            SMS_STS_OFS_L, 0, 2, "Position Correction (reset)", signed=True
        )
        motor6.lockEeprom()

        print("\n=== Step 4: Cap runtime torque for homing ===")
        motor6.setSetting(
            STS_TORQUE_LIMIT_L, HOMING_TORQUE_LIMIT, 2, "Runtime Torque Limit"
        )

        print("\n=== Step 5: Enable torque ===")
        motor6.setSetting(SMS_STS_TORQUE_ENABLE, 1, 1, "Torque Enable")
        time.sleep(1)

        startPosition = motor6.getPosition()
        print(f"\nStart position (raw, unwrapped): {startPosition}")

        telemetryPlot = Telemetry(
            title=f"Servo ID {PRISMATIC_ID} Sensorless Homing - Current vs Time",
            thresholdMa=SPIKE_CURRENT_THRESHOLD_MA,
        )
        telemetryPlot.markEvent("Start")

        # -----------------------------------------------------SEARCH / DRIVE HELPERS-----------------------------------------------------------

        def waitForStop():
            """Polls telemetry (feeding the live plot) until the motor reports moving==0 - i.e.
            the stop command issued after an obstacle detection has actually taken effect -
            then dwells SETTLE_DWELL_S longer so residual current decays. Returns the settled
            position (more accurate than the spike-instant position, since the joint keeps
            coasting into the stop for a moment after the goal is re-committed)."""
            telemetry = None

            settleDeadline = time.time() + STOP_SETTLE_TIMEOUT_S
            while time.time() < settleDeadline:
                sampleT0 = time.time()
                telemetry = motor6.getTelemetry()
                if telemetry is not None:
                    telemetryPlot.sample(telemetry["currentMa"])
                    if telemetry["moving"] == 0:
                        break
                dt = time.time() - sampleT0
                if dt < SAMPLE_PERIOD:
                    time.sleep(SAMPLE_PERIOD - dt)
            else:
                print(
                    f"  [WARN] motor still reports moving after {STOP_SETTLE_TIMEOUT_S}s settle timeout - proceeding anyway"
                )

            dwellDeadline = time.time() + SETTLE_DWELL_S
            while time.time() < dwellDeadline:
                sampleT0 = time.time()
                sampled = motor6.getTelemetry()
                if sampled is not None:
                    telemetryPlot.sample(sampled["currentMa"])
                    telemetry = sampled
                dt = time.time() - sampleT0
                if dt < SAMPLE_PERIOD:
                    time.sleep(SAMPLE_PERIOD - dt)

            return telemetry["position"] if telemetry is not None else None

        def searchForObstacle(directionLabel, searchGoalPosition, speed, acceleration):
            """Blocking move toward searchGoalPosition, sampling current every poll. The instant
            present current reaches SPIKE_CURRENT_THRESHOLD_MA, stops the goal-position command
            (by re-commanding the present position), waits for the joint to actually come to
            rest (waitForStop()), then retreats BACKOFF steps back the way it came and waits
            for that to settle too. Staying in contact with the obstacle keeps drawing current
            above threshold indefinitely -- re-commanding the present position as "stop" doesn't
            relieve a joint that's still pressed against something -- which otherwise makes the
            very next move misread that leftover contact current as a brand-new obstacle at zero
            travel distance. Returns the settled obstacle position (before the backoff).
            Raises HomingError if searchGoalPosition is reached without ever seeing a spike.
            """
            motor6.setPosition(searchGoalPosition, speed, acceleration)

            hasMoved = False
            while True:
                sampleT0 = time.time()
                telemetry = motor6.getTelemetry()

                if telemetry is not None:
                    telemetryPlot.sample(telemetry["currentMa"])
                    print(
                        f"  {directionLabel:10s} GoalPos:{searchGoalPosition:6d} PresPos:{telemetry['position']:6d} "
                        f"PresSpd:{telemetry['speed']:5d} Current:{telemetry['currentMa']:7.1f}mA"
                    )

                    if abs(telemetry["currentMa"]) >= SPIKE_CURRENT_THRESHOLD_MA:
                        motor6.setPosition(
                            telemetry["position"], speed, acceleration
                        )  # stop the goal-position command immediately
                        print(
                            f"  *** Obstacle detected {directionLabel} near position {telemetry['position']} "
                            f"(current {telemetry['currentMa']:.1f}mA >= {SPIKE_CURRENT_THRESHOLD_MA}mA) - settling ***"
                        )
                        obstaclePosition = waitForStop()
                        if obstaclePosition is None:
                            obstaclePosition = telemetry["position"]
                        print(f"  *** Settled at {obstaclePosition} ***")

                        backoffDirection = (
                            1 if searchGoalPosition > obstaclePosition else -1
                        )
                        backoffGoal = obstaclePosition - backoffDirection * BACKOFF
                        backoffLabel = "backward" if backoffDirection > 0 else "forward"
                        print(
                            f"  *** Backing off {BACKOFF} steps {backoffLabel} to {backoffGoal} "
                            f"so current returns to baseline before the next move ***"
                        )
                        motor6.setPosition(backoffGoal, speed, acceleration)
                        waitForStop()

                        return obstaclePosition

                    if telemetry["moving"] == 1:
                        hasMoved = True
                    if telemetry["moving"] == 0 and (
                        hasMoved
                        or abs(telemetry["position"] - searchGoalPosition)
                        <= SCS_MOVE_TOLERANCE
                    ):
                        raise HomingError(
                            f"No obstacle detected {directionLabel}: reached search goal {searchGoalPosition} "
                            f"without current ever reaching {SPIKE_CURRENT_THRESHOLD_MA}mA."
                        )

                dt = time.time() - sampleT0
                if dt < SAMPLE_PERIOD:
                    time.sleep(SAMPLE_PERIOD - dt)

        def driveToPosition(goalPosition, phaseLabel, speed, acceleration):
            """Blocking move to goalPosition (normal arrival detection), sampling current every
            poll for the live plot. Aborts with HomingError if current unexpectedly spikes
            mid-move - that would mean the target position isn't actually clear."""
            motor6.setPosition(goalPosition, speed, acceleration)

            hasMoved = False
            while True:
                sampleT0 = time.time()
                telemetry = motor6.getTelemetry()

                if telemetry is not None:
                    telemetryPlot.sample(telemetry["currentMa"])
                    print(
                        f"  {phaseLabel:14s} GoalPos:{goalPosition:6d} PresPos:{telemetry['position']:6d} "
                        f"PresSpd:{telemetry['speed']:5d} Current:{telemetry['currentMa']:7.1f}mA"
                    )

                    if abs(telemetry["currentMa"]) >= SPIKE_CURRENT_THRESHOLD_MA:
                        motor6.setPosition(telemetry["position"], speed, acceleration)
                        raise HomingError(
                            f"Unexpected obstacle during '{phaseLabel}' at position {telemetry['position']} "
                            f"(current {telemetry['currentMa']:.1f}mA >= {SPIKE_CURRENT_THRESHOLD_MA}mA)."
                        )

                    if telemetry["moving"] == 1:
                        hasMoved = True
                    if telemetry["moving"] == 0 and (
                        hasMoved
                        or abs(telemetry["position"] - goalPosition)
                        <= SCS_MOVE_TOLERANCE
                    ):
                        break

                dt = time.time() - sampleT0
                if dt < SAMPLE_PERIOD:
                    time.sleep(SAMPLE_PERIOD - dt)

        # -----------------------------------------------------HOMING SEQUENCE-----------------------------------------------------------

        print("\n=== Step 6: Search forward for obstacle #1 ===")
        telemetryPlot.markEvent("Obstacle Search #1")
        forwardPosition = searchForObstacle(
            "forward", startPosition + SEARCH_DISTANCE, homingSpeed, homingAcc
        )
        telemetryPlot.markEvent(f"Obstacle #1 @ {forwardPosition}")

        print("\n=== Step 7: Search backward for obstacle #2 ===")
        telemetryPlot.markEvent("Obstacle Search #2")
        backPosition = searchForObstacle(
            "backward", forwardPosition - SEARCH_DISTANCE, homingSpeed, homingAcc
        )
        telemetryPlot.markEvent(f"Obstacle #2 @ {backPosition}")

        centerPosition = round((backPosition + forwardPosition) / 2)
        print(f"\nBack obstacle (unwrapped):    {backPosition}")
        print(f"Forward obstacle (unwrapped): {forwardPosition}")
        print(f"Computed center (unwrapped):  {centerPosition}")

        print("\n=== Step 8: Drive to computed center ===")
        telemetryPlot.markEvent("Moving to Center")
        driveToPosition(centerPosition, "moving_to_center", homingSpeed, homingAcc)

        time.sleep(2)

        # -----------------------------------------------------TORQUE + BOUNDS HANDOFF-----------------------------------------------------------

        print("\n=== Step 9: Raise motor 6 runtime torque for its range test ===")
        motor6.setSetting(
            STS_TORQUE_LIMIT_L, movingTorqueLimit, 2, "Runtime Torque Limit"
        )

        print("\n=== Step 10: Cap motors 7-10 runtime torque for range test ===")
        for id in MOTOR_CONFIGS:
            motors[id].setSetting(
                STS_TORQUE_LIMIT_L, ROTARY_TORQUE_LIMIT, 2, "Runtime Torque Limit"
            )

        # Buffer motor 6's range-test bounds off the detected obstacles, same margin
        # prismaticJointCalibration.py's own range test uses, so movingSpeed/movingAcc doesn't
        # ram the stops.
        motor6.minPosition = backPosition + RANGE_BUFFER
        motor6.maxPosition = forwardPosition - RANGE_BUFFER

        centers = {PRISMATIC_ID: centerPosition}
        speeds = {PRISMATIC_ID: (movingSpeed, movingAcc)}
        for id, cfg in MOTOR_CONFIGS.items():
            centers[id] = SCS_CALIBRATED_CENTER
            speeds[id] = (cfg["moving_speed"], cfg["moving_acc"])

        # -----------------------------------------------------RANGE OF MOTION CHECK (ALL MOTORS)--------------------------------------------

        telemetryPlot.markEvent("Range Test: All Motors")
        time.sleep(5)

        print("--- Verifying Range of Motion ---")

        for id, motor in motors.items():
            print(f"--- Motor ID {id} ---")
            speed, acceleration = speeds[id]
            center = centers[id]

            print(f"Moving motor {id} to its minimum position...")
            motor.moveToPosition(motor.minPosition, speed, acceleration)
            time.sleep(2.0)

            print(f"Moving motor {id} to calibrated center...")
            motor.moveToPosition(center, speed, acceleration)
            time.sleep(2.0)

            print(f"Moving motor {id} to its maximum position...")
            motor.moveToPosition(motor.maxPosition, speed, acceleration)
            time.sleep(2.0)

            print(f"Moving motor {id} to calibrated center...")
            motor.moveToPosition(center, speed, acceleration)
            time.sleep(2.0)

        print("--- Range of Motion Check Complete --- \n")

        print("\n=== Homing + Range Test Complete ===")
        print(f"Back obstacle (unwrapped):    {backPosition}")
        print(f"Forward obstacle (unwrapped): {forwardPosition}")
        print(f"Center (unwrapped):           {centerPosition}")
        print(f"Motor {PRISMATIC_ID} range-test min (buffered): {motor6.minPosition}")
        print(f"Motor {PRISMATIC_ID} range-test max (buffered): {motor6.maxPosition}")
        print(
            f"Motor {PRISMATIC_ID} runtime torque limit: {movingTorqueLimit / 10.0:.1f}%"
        )
        print(f"Motors 7-10 runtime torque limit: {ROTARY_TORQUE_LIMIT / 10.0:.1f}%")

    except HomingError as e:
        print(f"\n!!! HOMING FAILED: {e} !!!")
        motor6.setSetting(SMS_STS_TORQUE_ENABLE, 0, 1, "Torque Enable")

    except KeyboardInterrupt:
        print("\n!!! Interrupted by user (Ctrl+C) !!!")
        motor6.setSetting(SMS_STS_TORQUE_ENABLE, 0, 1, "Torque Enable")

    finally:
        if telemetryPlot is not None:
            telemetryPlot.freeze()
            print("\nClose the plot window to exit.")
            telemetryPlot.show()
