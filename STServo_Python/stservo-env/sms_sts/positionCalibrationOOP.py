
#-----------------------------------------------------GENERAL IMPORTS----------------------------------------------------------

import time
from bus import Bus
print("General Imports Complete\n")

#-------------------------------------------------------INITIALIZE SERVO SETTINGS-------------------------------------------------

BAUDRATE                        = 1000000                           # SC Servo default baudrate : 1000000 (1 Mbps)
DEVICENAME                      = '/dev/tty.usbmodem5AAF2197651'    # Check which port is being used on your controller
                                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

SCS_CALIBRATED_CENTER           = 2048                              # The position value that becomes the "center" position after correction

# Per-motor configuration. Fill in the desired center rod position, range,
# moving speed, and moving acc for each of the four motors before running.
MOTOR_CONFIGS = {
    7:  {"desired_center_raw": 2750, "range": 700, "moving_speed": 1200, "moving_acc": 10, "tolerance": 3},
    8:  {"desired_center_raw": 1750, "range": 700, "moving_speed": 1200, "moving_acc": 10, "tolerance": 3},
    9:  {"desired_center_raw": 3385, "range": 1400, "moving_speed": 1200, "moving_acc": 10, "tolerance": 3},
    10: {"desired_center_raw": 3860, "range": 1400, "moving_speed": 1200, "moving_acc": 10, "tolerance": 3},
}

print(f"SC Servo BAUDRATE: {BAUDRATE}")
print(f"SC Servo DEVICENAME: {DEVICENAME}")
print(f"SC Servo Calibrated Center: {SCS_CALIBRATED_CENTER}")
print("")

#--------------------------------------------------BUS AND MOTOR INITIALIZATION-------------------------------------------------

bus = Bus(DEVICENAME, BAUDRATE)

# Build one Motor per entry in MOTOR_CONFIGS. angleResolution/positionMargin aren't used by this
# calibration flow, and minPositionLimit/maxPositionLimit just mirror the soft min/max position
# since this script never touches the hardware angle-limit registers.
motors = {}
for id, cfg in MOTOR_CONFIGS.items():
    minPosition = SCS_CALIBRATED_CENTER - cfg["range"]
    maxPosition = SCS_CALIBRATED_CENTER + cfg["range"]
    motors[id] = bus.addMotor(
        id=id,
        rawCenterPosition=cfg["desired_center_raw"],
        range=cfg["range"],
        minPosition=minPosition,
        maxPosition=maxPosition,
        speed=cfg["moving_speed"],
        acceleration=cfg["moving_acc"],
        positionTolerance=cfg["tolerance"],
        angleResolution=None,
        minPositionLimit=minPosition,
        maxPositionLimit=maxPosition,
        positionMargin=0,
    )

bus.printAllSettings()
for motor in motors.values():
    motor.printAllSettings()

#--------------------------------------------------------------MAIN-------------------------------------------------------------

with bus:

    #------------------------------------------------------------POSITION CORRECTION----------------------------------------------

    # Move all motors to their desired physical orientation first, simultaneously
    print("Moving all motors to their target physical center angle...")
    bus.moveMotorsToPositions(motors, {id: motor.rawCenterPosition for id, motor in motors.items()})
    time.sleep(2.0)  # Ensure they have completely stopped vibrating

    print("--- Starting Offset Calibration Process ---")

    for id, motor in motors.items():
        print(f"-- Motor ID {id} --")

        # Step 1: Unlock EEPROM Configuration Memory (Address 55 = 0)
        motor.unlockEeprom()
        time.sleep(0.1)

        # Step 2: Write the offset delta to Register 31 (2 Bytes)
        correction = motor.rawCenterPosition - SCS_CALIBRATED_CENTER
        motor.setSetting(31, correction, 2, "Position Offset (OFS)")
        time.sleep(0.1)

        # Step 3: Lock EEPROM back down (Address 55 = 1) to protect the memory
        motor.lockEeprom()
        time.sleep(0.2)

        # Step 4: Verify the offset was written successfully
        motor.getSetting(31, 2, "Verified Offset")

    # Step 5: Move all motors to the newly calibrated center and confirm they read 2048
    print("Moving all motors to calibrated center...")
    bus.moveMotorsToPositions(motors, {id: SCS_CALIBRATED_CENTER for id in motors})

    for id, motor in motors.items():
        print(f"[ID:{id:03d}] Present position after calibration: {motor.getPosition()}")

    print(f"--- Calibration Complete --- \n")
    time.sleep(2)

    #------------------------------------------------------------RANGE OF MOTION CHECK--------------------------------------------

    print("--- Verifying Range of Motion ---")

    print("Moving all motors to their minimum position...")
    bus.moveMotorsToPositions(motors, {id: motor.minPosition for id, motor in motors.items()})
    time.sleep(2.0)

    print("Moving all motors to calibrated center...")
    bus.moveMotorsToPositions(motors, {id: SCS_CALIBRATED_CENTER for id in motors})
    time.sleep(2.0)

    print("Moving all motors to their maximum position...")
    bus.moveMotorsToPositions(motors, {id: motor.maxPosition for id, motor in motors.items()})
    time.sleep(2.0)

    print("Moving all motors to calibrated center...")
    bus.moveMotorsToPositions(motors, {id: SCS_CALIBRATED_CENTER for id in motors})
    time.sleep(2.0)

    print("--- Range of Motion Check Complete --- \n")
