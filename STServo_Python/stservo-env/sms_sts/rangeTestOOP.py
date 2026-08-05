
#-----------------------------------------------------GENERAL IMPORTS----------------------------------------------------------

import time
from bus import Bus
print("General Imports Complete\n")

#-------------------------------------------------------INITIALIZE SERVO SETTINGS-------------------------------------------------

BAUDRATE                        = 1000000                           # SC Servo default baudrate : 1000000 (1 Mbps)
DEVICENAME                      = '/dev/tty.usbmodem5AAF2197651'    # Check which port is being used on your controller
                                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

SCS_CALIBRATED_CENTER           = 2048                              # The position value that becomes the "center" position after correction

# Per-motor configuration. This assumes motors have already been offset-calibrated
# (via positionCalibrationOOP.py) so that SCS_CALIBRATED_CENTER is their true center.
# "margin" separates the software position limits (minPosition/maxPosition) from the
# hardware backup limits (minPositionLimit/maxPositionLimit): the hardware limit is set
# `margin` steps wider on each side, so it only kicks in if the software limit fails.
MOTOR_CONFIGS = {
    # 7:  {"range": 700, "moving_speed": 1200, "moving_acc": 10, "tolerance": 3, "margin": 100},
    # 8:  {"range": 700, "moving_speed": 1200, "moving_acc": 10, "tolerance": 3, "margin": 100},
    # 9:  {"range": 1400, "moving_speed": 1200, "moving_acc": 10, "tolerance": 3, "margin": 100},
    # 10: {"range": 1400, "moving_speed": 1200, "moving_acc": 10, "tolerance": 3, "margin": 100},
    6:  {"range": 200, "moving_speed": 300, "moving_acc": 20, "tolerance": 3, "margin": 100, "angleResolution": 5},
}

print(f"SC Servo BAUDRATE: {BAUDRATE}")
print(f"SC Servo DEVICENAME: {DEVICENAME}")
print(f"SC Servo Calibrated Center: {SCS_CALIBRATED_CENTER}")
print("")

#--------------------------------------------------BUS AND MOTOR INITIALIZATION-------------------------------------------------

bus = Bus(DEVICENAME, BAUDRATE)

#--------------------------------------------------------------MAIN-------------------------------------------------------------

with bus:

    # Build one Motor per entry in MOTOR_CONFIGS. rawCenterPosition is just SCS_CALIBRATED_CENTER
    # since these motors are assumed to already be offset-calibrated. minPositionLimit/
    # maxPositionLimit are set `margin` steps wider than minPosition/maxPosition so the hardware
    # limit is a real backup rather than a mirror of the software limit. Construction happens
    # here (after the port is open) rather than before `with bus:`, because a motor with a
    # non-default angleResolution (ID 6) writes to hardware from inside its own constructor.
    motors = {}
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
            speed=cfg["moving_speed"],
            acceleration=cfg["moving_acc"],
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
    # each servo's EEPROM before running the range check.
    print("--- Applying Hardware Position Limits ---")
    for motor in motors.values():
        motor.applyPositionLimits()

    #------------------------------------------------------------RANGE OF MOTION CHECK--------------------------------------------
    time.sleep(20)

    print("--- Verifying Range of Motion ---")

    for id, motor in motors.items():
        print(f"--- Motor ID {id} ---")

        print(f"Moving motor {id} to its minimum position...")
        motor.moveToPosition(motor.minPosition)
        time.sleep(2.0)

        print(f"Moving motor {id} to calibrated center...")
        motor.moveToPosition(SCS_CALIBRATED_CENTER)
        time.sleep(2.0)

        print(f"Moving motor {id} to its maximum position...")
        motor.moveToPosition(motor.maxPosition)
        time.sleep(2.0)

        print(f"Moving motor {id} to calibrated center...")
        motor.moveToPosition(SCS_CALIBRATED_CENTER)
        time.sleep(2.0)

    print("--- Range of Motion Check Complete --- \n")
