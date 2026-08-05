
#-----------------------------------------------------GENERAL IMPORTS----------------------------------------------------------

import time
from bus import Bus
from telemetry import Telemetry
from scservo_sdk import *                      # Uses SC Servo SDK library
print("General Imports Complete\n")

#-------------------------------------------------------INITIALIZE SERVO SETTINGS-------------------------------------------------

SCS_ID                           = 6                                 # CHANGE THIS to the ID of the motor you want to calibrate
BAUDRATE                         = 1000000                           # SC Servo default baudrate : 1000000 (1 Mbps)
DEVICENAME                       = '/dev/tty.usbmodem5AAF2197651'    # Check which port is being used on your controller

SCS_MOVING_SPEED                 = 300                               # Matches the run that produced the current-draw baseline below (prismaticCurrentDraw.py)
SCS_MOVING_ACC                   = 30
SCS_MOVE_TOLERANCE               = 3                                 # Steps within which we consider a move "arrived"

# This joint is driven by a lead screw, so it needs several full servo revolutions to cover
# its prismatic travel. SEARCH_DISTANCE just has to be comfortably larger than the true
# travel so each search leg is guaranteed to hit its mechanical stop before running out of
# commanded distance - it does NOT need to be a tight bound (searchForObstacle() below stops
# the instant a current spike is seen, long before actually reaching this goal).
SEARCH_DISTANCE                  = 20000

SAMPLE_HZ                        = 30                                # Telemetry sampling rate during motion
SAMPLE_PERIOD                    = 1.0 / SAMPLE_HZ

# From prismaticCurrentDraw.py's free-travel current characterization at this same
# speed/acc (SCS_ID 6, angle resolution 1): baseline mean 11.4mA, stdev 3.9mA, peak 19.5mA
# across forward+backward legs -> mean + 4*stdev ~= 27mA. Comfortably above the observed
# free-travel peak, well below what a real stall against the mechanical stop produces.
SPIKE_CURRENT_THRESHOLD_MA        = 27

# After a spike stops the goal-position command, the joint keeps coasting/decelerating for a
# moment before it's actually at rest - starting the next leg's motion during that window
# reverses direction while still moving, which itself draws a current spike from braking, not
# from a real second obstacle. waitForStop() below polls until moving==0 (or this timeout),
# then dwells an extra beat so residual current has time to decay before the next leg begins.
STOP_SETTLE_TIMEOUT_S              = 2.0
SETTLE_DWELL_S                     = 0.3

# SRAM, 2 bytes, 0.1%/count - runtime torque cap (address 48). Not exposed by name in this
# SDK, but matches the standard FeeTech STS/SMS memory table (same table that gives
# MIN_ANGLE_LIMIT_L=9, OFS_L=31, MODE=33, TORQUE_ENABLE=40, PRESENT_CURRENT_L=69, ...
# all already defined by this SDK).
STS_TORQUE_LIMIT_L               = 48
HOMING_TORQUE_LIMIT               = 125                               # 12.5% of max torque - keeps stop impacts gentle during search
FULL_TORQUE_LIMIT                 = 1000                              # 100.0% - restored once homing/centering is complete

print(f"SC Servo ID: {SCS_ID}")
print(f"SC Servo BAUDRATE: {BAUDRATE}")
print(f"SC Servo DEVICENAME: {DEVICENAME}")
print(f"SC Servo Moving Speed: {SCS_MOVING_SPEED}")
print(f"SC Servo Moving Acc: {SCS_MOVING_ACC}")
print(f"Search Distance: {SEARCH_DISTANCE}")
print(f"Spike Current Threshold (mA): {SPIKE_CURRENT_THRESHOLD_MA}")
print(f"Homing Torque Limit: {HOMING_TORQUE_LIMIT / 10.0:.1f}%")
print("")

#--------------------------------------------------BUS AND MOTOR INITIALIZATION-------------------------------------------------

bus = Bus(DEVICENAME, BAUDRATE)


class HomingError(Exception):
    """Raised when a search leg fails to find an obstacle within SEARCH_DISTANCE, or an
    unexpected current spike occurs while driving to the computed center."""


with bus:

    motor = bus.addMotor(
        id=SCS_ID,
        rawCenterPosition=0,
        range=SEARCH_DISTANCE,
        minPosition=-SEARCH_DISTANCE,
        maxPosition=SEARCH_DISTANCE,
        speed=SCS_MOVING_SPEED,
        acceleration=SCS_MOVING_ACC,
        positionTolerance=SCS_MOVE_TOLERANCE,
        minPositionLimit=0,
        maxPositionLimit=0,        # 0/0 = unrestricted multi-turn rotation (no clipping at +/-4095)
        positionMargin=0,
        angleResolution=1,
    )

    telemetryPlot = None
    backPosition = None
    forwardPosition = None

    try:
        print("\n=== Step 1: Multi-turn Phase bit set (Angle Resolution stays 1) ===")
        motor.applyAngleResolution()   # unlocks EEPROM, re-asserts Angle Resolution=1, sets Phase BIT4, re-locks

        print("\n=== Step 2: Min/Max Angle Limit -> unrestricted (multi-turn) ===")
        motor.applyPositionLimits()

        print("\n=== Step 3: Reset Position Correction to 0 ===")
        motor.unlockEeprom()
        # Position Correction (address 31) is only ever reset to 0 here, so the default bit-15
        # signed encoding in setSetting is fine - 0 is unaffected by which bit is treated as sign.
        motor.setSetting(SMS_STS_OFS_L, 0, 2, "Position Correction (reset)", signed=True)
        motor.lockEeprom()

        print("\n=== Step 4: Cap runtime torque for homing ===")
        motor.setSetting(STS_TORQUE_LIMIT_L, HOMING_TORQUE_LIMIT, 2, "Runtime Torque Limit")

        print("\n=== Step 5: Enable torque ===")
        motor.setSetting(SMS_STS_TORQUE_ENABLE, 1, 1, "Torque Enable")
        time.sleep(1)

        startPosition = motor.getPosition()
        print(f"\nStart position (raw, unwrapped): {startPosition}")

        telemetryPlot = Telemetry(
            title=f"Servo ID {SCS_ID} Sensorless Homing - Current vs Time",
            thresholdMa=SPIKE_CURRENT_THRESHOLD_MA,
        )
        telemetryPlot.markEvent("Start")

        #-----------------------------------------------------SEARCH / DRIVE HELPERS-----------------------------------------------------------

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
                telemetry = motor.getTelemetry()
                if telemetry is not None:
                    telemetryPlot.sample(telemetry["currentMa"])
                    if telemetry["moving"] == 0:
                        break
                dt = time.time() - sampleT0
                if dt < SAMPLE_PERIOD:
                    time.sleep(SAMPLE_PERIOD - dt)
            else:
                print(f"  [WARN] motor still reports moving after {STOP_SETTLE_TIMEOUT_S}s settle timeout - proceeding anyway")

            dwellDeadline = time.time() + SETTLE_DWELL_S
            while time.time() < dwellDeadline:
                sampleT0 = time.time()
                sampled = motor.getTelemetry()
                if sampled is not None:
                    telemetryPlot.sample(sampled["currentMa"])
                    telemetry = sampled
                dt = time.time() - sampleT0
                if dt < SAMPLE_PERIOD:
                    time.sleep(SAMPLE_PERIOD - dt)

            return telemetry["position"] if telemetry is not None else None

        def searchForObstacle(directionLabel, searchGoalPosition):
            """Blocking move toward searchGoalPosition, sampling current every poll. The instant
            present current reaches SPIKE_CURRENT_THRESHOLD_MA, stops the goal-position command
            (by re-commanding the present position), waits for the joint to actually come to
            rest (waitForStop()), and returns the settled unwrapped position.
            Raises HomingError if searchGoalPosition is reached without ever seeing a spike."""
            motor.setPosition(searchGoalPosition)

            hasMoved = False
            while True:
                sampleT0 = time.time()
                telemetry = motor.getTelemetry()

                if telemetry is not None:
                    telemetryPlot.sample(telemetry["currentMa"])
                    print(f"  {directionLabel:10s} GoalPos:{searchGoalPosition:6d} PresPos:{telemetry['position']:6d} "
                          f"PresSpd:{telemetry['speed']:5d} Current:{telemetry['currentMa']:7.1f}mA")

                    if abs(telemetry["currentMa"]) >= SPIKE_CURRENT_THRESHOLD_MA:
                        motor.setPosition(telemetry["position"])   # stop the goal-position command immediately
                        print(f"  *** Obstacle detected {directionLabel} near position {telemetry['position']} "
                              f"(current {telemetry['currentMa']:.1f}mA >= {SPIKE_CURRENT_THRESHOLD_MA}mA) - settling ***")
                        obstaclePosition = waitForStop()
                        if obstaclePosition is None:
                            obstaclePosition = telemetry["position"]
                        print(f"  *** Settled at {obstaclePosition} ***")
                        return obstaclePosition

                    if telemetry["moving"] == 1:
                        hasMoved = True
                    if telemetry["moving"] == 0 and (hasMoved or abs(telemetry["position"] - searchGoalPosition) <= SCS_MOVE_TOLERANCE):
                        raise HomingError(
                            f"No obstacle detected {directionLabel}: reached search goal {searchGoalPosition} "
                            f"without current ever reaching {SPIKE_CURRENT_THRESHOLD_MA}mA."
                        )

                dt = time.time() - sampleT0
                if dt < SAMPLE_PERIOD:
                    time.sleep(SAMPLE_PERIOD - dt)

        def driveToPosition(goalPosition, phaseLabel):
            """Blocking move to goalPosition (normal arrival detection), sampling current every
            poll for the live plot. Aborts with HomingError if current unexpectedly spikes
            mid-move - that would mean the computed center isn't actually clear."""
            motor.setPosition(goalPosition)

            hasMoved = False
            while True:
                sampleT0 = time.time()
                telemetry = motor.getTelemetry()

                if telemetry is not None:
                    telemetryPlot.sample(telemetry["currentMa"])
                    print(f"  {phaseLabel:14s} GoalPos:{goalPosition:6d} PresPos:{telemetry['position']:6d} "
                          f"PresSpd:{telemetry['speed']:5d} Current:{telemetry['currentMa']:7.1f}mA")

                    if abs(telemetry["currentMa"]) >= SPIKE_CURRENT_THRESHOLD_MA:
                        motor.setPosition(telemetry["position"])
                        raise HomingError(
                            f"Unexpected obstacle during '{phaseLabel}' at position {telemetry['position']} "
                            f"(current {telemetry['currentMa']:.1f}mA >= {SPIKE_CURRENT_THRESHOLD_MA}mA)."
                        )

                    if telemetry["moving"] == 1:
                        hasMoved = True
                    if telemetry["moving"] == 0 and (hasMoved or abs(telemetry["position"] - goalPosition) <= SCS_MOVE_TOLERANCE):
                        break

                dt = time.time() - sampleT0
                if dt < SAMPLE_PERIOD:
                    time.sleep(SAMPLE_PERIOD - dt)

        #-----------------------------------------------------HOMING SEQUENCE-----------------------------------------------------------

        print("\n=== Step 6: Search forward for obstacle #1 ===")
        telemetryPlot.markEvent("Obstacle Search #1")
        forwardPosition = searchForObstacle("forward", startPosition + SEARCH_DISTANCE)
        telemetryPlot.markEvent(f"Obstacle #1 @ {forwardPosition}")

        print("\n=== Step 7: Search backward for obstacle #2 ===")
        telemetryPlot.markEvent("Obstacle Search #2")
        backPosition = searchForObstacle("backward", forwardPosition - SEARCH_DISTANCE)
        telemetryPlot.markEvent(f"Obstacle #2 @ {backPosition}")

        centerPosition = round((backPosition + forwardPosition) / 2)
        print(f"\nBack obstacle (unwrapped):    {backPosition}")
        print(f"Forward obstacle (unwrapped): {forwardPosition}")
        print(f"Computed center (unwrapped):  {centerPosition}")

        print("\n=== Step 8: Drive to computed center ===")
        telemetryPlot.markEvent("Moving to Center")
        driveToPosition(centerPosition, "moving_to_center")

        print("\n=== Step 9: Restore full runtime torque limit ===")
        motor.setSetting(STS_TORQUE_LIMIT_L, FULL_TORQUE_LIMIT, 2, "Runtime Torque Limit")

        print("\n=== Homing Complete ===")
        print(f"Back obstacle (unwrapped):    {backPosition}")
        print(f"Forward obstacle (unwrapped): {forwardPosition}")
        print(f"Center (unwrapped):           {centerPosition}")

    except HomingError as e:
        print(f"\n!!! HOMING FAILED: {e} !!!")
        motor.setSetting(SMS_STS_TORQUE_ENABLE, 0, 1, "Torque Enable")

    except KeyboardInterrupt:
        print("\n!!! Interrupted by user (Ctrl+C) !!!")
        motor.setSetting(SMS_STS_TORQUE_ENABLE, 0, 1, "Torque Enable")

    finally:
        if telemetryPlot is not None:
            telemetryPlot.freeze()
            print("\nClose the plot window to exit.")
            telemetryPlot.show()
