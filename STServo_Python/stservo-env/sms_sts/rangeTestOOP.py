
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
MOTOR_CONFIGS = {
    7:  {"range": 700, "moving_speed": 1200, "moving_acc": 10, "tolerance": 3},
    8:  {"range": 700, "moving_speed": 1200, "moving_acc": 10, "tolerance": 3},
    9:  {"range": 1400, "moving_speed": 1200, "moving_acc": 10, "tolerance": 3},
    10: {"range": 1400, "moving_speed": 1200, "moving_acc": 10, "tolerance": 3},
}

print(f"SC Servo BAUDRATE: {BAUDRATE}")
print(f"SC Servo DEVICENAME: {DEVICENAME}")
print(f"SC Servo Calibrated Center: {SCS_CALIBRATED_CENTER}")
print("")

#--------------------------------------------------BUS AND MOTOR INITIALIZATION-------------------------------------------------

bus = Bus(DEVICENAME, BAUDRATE)

# Build one Motor per entry in MOTOR_CONFIGS. rawCenterPosition is just SCS_CALIBRATED_CENTER since
# these motors are assumed to already be offset-calibrated. angleResolution/positionMargin aren't
# used by this range check, and minPositionLimit/maxPositionLimit just mirror the soft min/max
# position since this script never touches the hardware angle-limit registers.
motors = {}
for id, cfg in MOTOR_CONFIGS.items():
    minPosition = SCS_CALIBRATED_CENTER - cfg["range"]
    maxPosition = SCS_CALIBRATED_CENTER + cfg["range"]
    motors[id] = bus.addMotor(
        id=id,
        rawCenterPosition=SCS_CALIBRATED_CENTER,
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

    #------------------------------------------------------------RANGE OF MOTION CHECK--------------------------------------------

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
