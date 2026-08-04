
#-----------------------------------------------------GENERAL IMPORTS----------------------------------------------------------

import sys
import time
sys.path.append("..")
from scservo_sdk import *                      # Uses SC Servo SDK library
from bus import Bus

#-------------------------------------------------------INITIALIZE SERVO SETTINGS-------------------------------------------------

# Default setting
SCS_ID                          = 1                                 # SC Servo ID : 1
BAUDRATE                        = 1000000                           # SC Servo default baudrate : 1000000 (1 Mbps)
DEVICENAME                      = '/dev/tty.usbmodem5AAF2197651'    # Check which port is being used on your controller
                                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

print(f"SC Servo ID: {SCS_ID}")
print(f"SC Servo BAUDRATE: {BAUDRATE}")
print(f"SC Servo DEVICENAME: {DEVICENAME}")
print("")

#-------------------------------------------------------MOVE TO POSITION HELPER-----------------------------------------------

def moveToPosition(motor, goalPosition, speed, acceleration, tolerance):
    motor.setPosition(goalPosition, speed, acceleration)

    hasMoved = False
    while True:
        presentPosition, presentSpeed = motor.getPositionSpeed()
        print(f"[ID:{motor.id:03d}] GoalPos:{goalPosition} PresPos:{presentPosition} PresSpd:{presentSpeed}")

        moving = motor.isMoving()
        if moving:
            hasMoved = True

        # Trust moving==False as "arrived" once we've seen it move, OR if we're already
        # within the servo's dead zone (it may never report moving if the goal is
        # close enough that it decides no move is needed).
        if not moving and (hasMoved or abs(presentPosition - goalPosition) <= tolerance):
            break

#--------------------------------------------------BUS AND MOTOR INITIALIZATION-------------------------------------

with Bus(DEVICENAME, BAUDRATE) as bus:

    motor = bus.addMotor(
        id=SCS_ID,
        rawCenterPosition=0,
        range=4096,
        minPosition=0,
        maxPosition=4096,
        speed=1200,
        acceleration=10,
        positionTolerance=1,
        angleResolution=4,
        minPositionLimit=0,
        maxPositionLimit=0,
        positionMargin=0,
    )

    #---------------------------------------------------------SERVO SETTINGS------------------------------------------------------

    # EEPROM must be unlocked before these addresses can be written
    motor.unlockEeprom()

    #----------------------------------Apply Settings

    motor.setSetting(SMS_STS_TORQUE_ENABLE, 1, 1, "Torque Switch")
    motor.setSetting(SMS_STS_MODE, 0, 1, "Operation Mode")
    motor.setSetting(30, motor.angleResolution, 1, "Angle Resolution")
    # Both limits must be exactly 0 (not 0/4095) to enable multi-turn mode - 0/4095 is
    # still just the bounded single-turn range and clamps Goal Position regardless of
    # Angle Resolution.
    motor.setSetting(SMS_STS_MIN_ANGLE_LIMIT_L, motor.minPositionLimit, 2, "Min Angle Limit", signed=True)
    motor.setSetting(SMS_STS_MAX_ANGLE_LIMIT_L, motor.maxPositionLimit, 2, "Max Angle Limit", signed=True)

    # Multi-turn also needs Phase (0x12/18) BIT4 set so Present Position feedback reflects
    # the larger angle instead of wrapping every revolution. Read-modify-write so we don't
    # clobber whatever other bits (e.g. direction) are already set in this byte.
    phase = motor.getSetting(18, 1, "Phase (before)")
    motor.setSetting(18, phase | 0x10, 1, "Phase (multi-turn bit4 set)")

    #-----------------------------------------------

    # Re-lock EEPROM to protect the settings just written
    motor.lockEeprom()

    #----------------------------------Verify Settings

    motor.getSetting(30, 1, "Angle Resolution")
    motor.getSetting(SMS_STS_MODE, 1, "Operation Mode")
    motor.setSetting(SMS_STS_TORQUE_ENABLE, 1, 1, "Torque Switch")
    motor.getSetting(SMS_STS_MIN_ANGLE_LIMIT_L, 2, "Min Angle Limit")
    motor.getSetting(SMS_STS_MAX_ANGLE_LIMIT_L, 2, "Max Angle Limit")
    motor.getSetting(18, 1, "Phase")

    #------------------------------------------------

    time.sleep(5)

    #------------------------------------------------------------MAIN CONTROL LOOP------------------------------------------------

    # move to "start position"
    moveToPosition(motor, 0, motor.speed, motor.acceleration, motor.positionTolerance)
    time.sleep(2)
    moveToPosition(motor, 4096, motor.speed, motor.acceleration, motor.positionTolerance)
    time.sleep(2)
    moveToPosition(motor, 0, motor.speed, motor.acceleration, motor.positionTolerance)
