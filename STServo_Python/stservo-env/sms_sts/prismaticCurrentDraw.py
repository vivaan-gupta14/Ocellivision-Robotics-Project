
#-----------------------------------------------------GENERAL IMPORTS----------------------------------------------------------

import time
import statistics
from bus import Bus
from motor import Motor
from scservo_sdk import *                      # Uses SC Servo SDK library
print("General Imports Complete\n")

#-------------------------------------------------------INITIALIZE SERVO SETTINGS-------------------------------------------------

SCS_ID                           = 6                                 # CHANGE THIS to the ID of the motor you want to test
BAUDRATE                         = 1000000                           # SC Servo default baudrate : 1000000 (1 Mbps)
DEVICENAME                       = '/dev/tty.usbmodem5AAF2197651'    # Check which port is being used on your controller

SCS_MOVING_SPEED                 = 300                              # SC Servo position-mode travel speed (steps/sec)
SCS_MOVING_ACC                   = 30                                # SC Servo moving acc (steps/sec^2)
SCS_MOVE_TOLERANCE               = 3                                 # Steps within which we consider a move "arrived"

STEP_DISTANCE                    = 4096                              # Steps to travel each leg (forward, then back)
SAMPLE_HZ                        = 30                                # Telemetry sampling rate during motion
SAMPLE_PERIOD                    = 1.0 / SAMPLE_HZ

print(f"SC Servo ID: {SCS_ID}")
print(f"SC Servo BAUDRATE: {BAUDRATE}")
print(f"SC Servo DEVICENAME: {DEVICENAME}")
print(f"Step Distance: {STEP_DISTANCE}")
print(f"Sample Rate (Hz): {SAMPLE_HZ}")
print("")

#--------------------------------------------------BUS AND MOTOR INITIALIZATION-------------------------------------------------

bus = Bus(DEVICENAME, BAUDRATE)

with bus:

    motor = bus.addMotor(
        id=SCS_ID,
        rawCenterPosition=0,
        range=STEP_DISTANCE,
        minPosition=-STEP_DISTANCE,
        maxPosition=STEP_DISTANCE,
        positionTolerance=SCS_MOVE_TOLERANCE,
        minPositionLimit=0,
        maxPositionLimit=0,        # 0/0 = unrestricted multi-turn rotation (no clipping at +/-4095)
        positionMargin=0,
        angleResolution=1,
    )

    print("\n=== Step 1: Multi-turn Phase bit set (Angle Resolution stays 1) ===")
    motor.applyAngleResolution()   # unlocks EEPROM, re-asserts Angle Resolution=1, sets Phase BIT4, re-locks

    print("\n=== Step 2: Min/Max Angle Limit -> unrestricted (multi-turn) ===")
    motor.applyPositionLimits()

    print("\n=== Step 3: Enable torque ===")
    motor.setSetting(SMS_STS_TORQUE_ENABLE, 1, 1, "Torque Enable")
    time.sleep(1)

    startPosition = motor.getPosition()
    print(f"\nStart position (raw, unwrapped): {startPosition}")

    #-----------------------------------------------------MOVE WITH CURRENT SAMPLING-----------------------------------------------------------

    samples = []   # list of (phaseLabel, elapsedSeconds, currentMa)
    t0 = time.time()

    def moveWithCurrentSampling(goalPosition, phaseLabel):
        motor.setPosition(goalPosition, SCS_MOVING_SPEED, SCS_MOVING_ACC)

        hasMoved = False
        while True:
            sampleT0 = time.time()

            telemetry = motor.getTelemetry()
            if telemetry is not None:
                elapsed = time.time() - t0
                samples.append((phaseLabel, elapsed, telemetry["currentMa"]))
                print(f"  {phaseLabel:10s} GoalPos:{goalPosition:6d} PresPos:{telemetry['position']:6d} "
                      f"PresSpd:{telemetry['speed']:5d} Current:{telemetry['currentMa']:7.1f}mA Moving:{telemetry['moving']}")

                if telemetry["moving"] == 1:
                    hasMoved = True

                if telemetry["moving"] == 0 and (hasMoved or abs(telemetry["position"] - goalPosition) <= SCS_MOVE_TOLERANCE):
                    break

            dt = time.time() - sampleT0
            if dt < SAMPLE_PERIOD:
                time.sleep(SAMPLE_PERIOD - dt)

    print(f"\n=== Step 4: Move forward {STEP_DISTANCE} steps ===")
    moveWithCurrentSampling(startPosition + STEP_DISTANCE, "forward")

    print(f"\n=== Step 5: Move back {STEP_DISTANCE} steps ===")
    moveWithCurrentSampling(startPosition, "backward")

    print("\n=== Step 6: Disable torque ===")
    motor.setSetting(SMS_STS_TORQUE_ENABLE, 0, 1, "Torque Enable")

    #-----------------------------------------------------SUMMARY-----------------------------------------------------------

    print("\n=== Current Draw Summary ===")
    for phaseLabel in ("forward", "backward"):
        phaseMagnitudes = [abs(current) for phase, elapsed, current in samples if phase == phaseLabel]
        if not phaseMagnitudes:
            continue
        print(f"  {phaseLabel:10s} mean={statistics.mean(phaseMagnitudes):7.1f}mA  "
              f"stdev={statistics.pstdev(phaseMagnitudes):7.1f}mA  peak={max(phaseMagnitudes):7.1f}mA  n={len(phaseMagnitudes)}")

    allMagnitudes = [abs(current) for phase, elapsed, current in samples]
    overallMean  = statistics.mean(allMagnitudes)
    overallStdev = statistics.pstdev(allMagnitudes)
    overallPeak  = max(allMagnitudes)

    print(f"\n  {'overall':10s} mean={overallMean:7.1f}mA  stdev={overallStdev:7.1f}mA  peak={overallPeak:7.1f}mA  n={len(allMagnitudes)}")

    suggestedMargin = overallMean + 4 * overallStdev
    print(f"\nSuggested obstacle-detection threshold: ~{suggestedMargin:.0f}mA")
    print(f"  (baseline mean {overallMean:.0f}mA + 4x stdev {overallStdev:.0f}mA of unobstructed travel; "
          f"observed peak during this free run was {overallPeak:.0f}mA)")
    print("  Pick a homing threshold comfortably above that peak so normal accel/decel transients")
    print("  never false-trigger, but still low enough to stop quickly once the joint actually stalls.")
