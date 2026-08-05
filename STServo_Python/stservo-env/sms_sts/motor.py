
#-----------------------------------------------------GENERAL IMPORTS----------------------------------------------------------

import sys
sys.path.append("..")
from scservo_sdk import *                      # Uses SC Servo SDK library

# Extra EEPROM addresses not already defined in scservo_sdk/sms_sts.py's control table
SMS_STS_ANGLE_RESOLUTION = 30
SMS_STS_PHASE            = 18

# Present_Position(56)..Present_Current(70) are contiguous in the memory table, so one raw
# read grabs position/speed/load/voltage/temperature/moving/current in a single serial transaction.
TELEMETRY_READ_LENGTH     = 15

#-----------------------------------------------------------MOTOR CLASS---------------------------------------------------------

class Motor:

    def __init__(self, packetHandler, id, rawCenterPosition, range, minPosition, maxPosition,
                 speed, acceleration, positionTolerance,
                 minPositionLimit, maxPositionLimit, positionMargin, angleResolution=1):

        self.packetHandler      = packetHandler        # Shared sms_sts packet handler for the bus this motor is on

        self.id                 = id                   # SC Servo ID
        self.rawCenterPosition  = rawCenterPosition     # Calibrated raw center position (post-offset-correction)
        self.range              = range                 # Max offset from center this motor is allowed to travel
        self.minPosition        = minPosition           # Soft minimum goal position (center - range)
        self.maxPosition        = maxPosition           # Soft maximum goal position (center + range)
        self.speed              = speed                 # Default moving speed (steps/sec)
        self.acceleration       = acceleration           # Default moving acceleration (steps/sec^2)
        self.positionTolerance  = positionTolerance     # Dead-zone: positions within this range count as "arrived"
        self.angleResolution    = angleResolution        # Configured Angle Resolution setting (EEPROM address 30)
        self.minPositionLimit   = minPositionLimit      # Hardware Min_Angle_Limit written to EEPROM
        self.maxPositionLimit   = maxPositionLimit      # Hardware Max_Angle_Limit written to EEPROM
        self.positionMargin     = positionMargin         # Margin added/subtracted between soft limits and hardware limits

        # Only non-default Angle Resolution needs to be pushed to hardware - the servo's
        # factory default is already 1, so motors that don't specify one stay untouched.
        if self.angleResolution != 1:
            self.applyAngleResolution()

    #---------------------------------------------------GET AND SET SETTINGS METHODS-----------------------------------------------------------

    def getSetting(self, address, numBytes, name, signed=False):
        if numBytes == 1:
            value, scsCommResult, scsError = self.packetHandler.read1ByteTxRx(self.id, address)
        else:
            value, scsCommResult, scsError = self.packetHandler.read2ByteTxRx(self.id, address)

        if scsCommResult != COMM_SUCCESS:
            print(f"[ID:{self.id:03d}][{name}] {self.packetHandler.getTxRxResult(scsCommResult)}")
            return None
        elif scsError != 0:
            print(f"[ID:{self.id:03d}][{name}] {self.packetHandler.getRxPacketError(scsError)}")
            return None

        value = self.packetHandler.scs_tohost(value, 15) if signed else value
        print(f"[ID:{self.id:03d}][{name}] Read {value}")
        return value

    def setSetting(self, address, value, numBytes, name, signed=False):
        if numBytes == 1:
            scsCommResult, scsError = self.packetHandler.write1ByteTxRx(self.id, address, value)
        else:
            wireValue = self.packetHandler.scs_toscs(value, 15) if signed else value
            scsCommResult, scsError = self.packetHandler.write2ByteTxRx(self.id, address, wireValue)

        if scsCommResult != COMM_SUCCESS:
            print(f"[ID:{self.id:03d}][{name}] {self.packetHandler.getTxRxResult(scsCommResult)}")
        elif scsError != 0:
            print(f"[ID:{self.id:03d}][{name}] {self.packetHandler.getRxPacketError(scsError)}")
        else:
            print(f"[ID:{self.id:03d}][{name}] Set to {value}")

    #-----------------------------------------------------EEPROM LOCK METHODS-----------------------------------------------------------

    def unlockEeprom(self):
        return self.packetHandler.unLockEprom(self.id)

    def lockEeprom(self):
        return self.packetHandler.LockEprom(self.id)

    #-----------------------------------------------------POSITION METHODS-----------------------------------------------------------

    def getPosition(self):
        scsPresentPosition, scsCommResult, scsError = self.packetHandler.ReadPos(self.id)
        if scsCommResult != COMM_SUCCESS:
            print(f"[ID:{self.id:03d}][Position] {self.packetHandler.getTxRxResult(scsCommResult)}")
        elif scsError != 0:
            print(f"[ID:{self.id:03d}][Position] {self.packetHandler.getRxPacketError(scsError)}")
        return scsPresentPosition

    def getPositionSpeed(self):
        scsPresentPosition, scsPresentSpeed, scsCommResult, scsError = self.packetHandler.ReadPosSpeed(self.id)
        if scsCommResult != COMM_SUCCESS:
            print(f"[ID:{self.id:03d}][PositionSpeed] {self.packetHandler.getTxRxResult(scsCommResult)}")
        elif scsError != 0:
            print(f"[ID:{self.id:03d}][PositionSpeed] {self.packetHandler.getRxPacketError(scsError)}")
        return scsPresentPosition, scsPresentSpeed

    def setPosition(self, goalPosition, speed=None, acceleration=None):
        scsCommResult, scsError = self.packetHandler.WritePosEx(
            self.id,
            goalPosition,
            speed if speed is not None else self.speed,
            acceleration if acceleration is not None else self.acceleration,
        )
        if scsCommResult != COMM_SUCCESS:
            print(f"[ID:{self.id:03d}][SetPosition] {self.packetHandler.getTxRxResult(scsCommResult)}")
        elif scsError != 0:
            print(f"[ID:{self.id:03d}][SetPosition] {self.packetHandler.getRxPacketError(scsError)}")

    def getTelemetry(self):
        # Single combined read of every present-value register, decoded the same way
        # torque_characterization_test.py's read_telemetry() does. Returns None on comm/servo error.
        data, scsCommResult, scsError = self.packetHandler.readTxRx(self.id, SMS_STS_PRESENT_POSITION_L, TELEMETRY_READ_LENGTH)
        if scsCommResult != COMM_SUCCESS or scsError != 0 or len(data) < TELEMETRY_READ_LENGTH:
            return None

        def word(offset):
            return self.packetHandler.scs_makeword(data[offset], data[offset + 1])

        return {
            "position":    self.packetHandler.scs_tohost(word(0), 15),
            "speed":       self.packetHandler.scs_tohost(word(2), 15),
            "loadPct":     self.packetHandler.scs_tohost(word(4), 10) / 10.0,   # 0.1%-of-max-torque units, bit10 = direction
            "voltage":     data[6] / 10.0,                                      # 0.1V units
            "temperature": data[7],
            "moving":      data[10],
            "currentMa":   self.packetHandler.scs_tohost(word(13), 15) * 6.5,   # 6.5mA units
        }

    def isMoving(self):
        moving, scsCommResult, scsError = self.packetHandler.ReadMoving(self.id)
        if scsCommResult != COMM_SUCCESS:
            print(f"[ID:{self.id:03d}][Moving] {self.packetHandler.getTxRxResult(scsCommResult)}")
        elif scsError != 0:
            print(f"[ID:{self.id:03d}][Moving] {self.packetHandler.getRxPacketError(scsError)}")
        return moving == 1

    def moveToPosition(self, goalPosition, speed=None, acceleration=None):
        # Blocking single-motor move: writes the goal position, then polls present position/speed
        # and moving status until the motor has arrived (or was already within positionTolerance).
        self.setPosition(goalPosition, speed, acceleration)

        hasMoved = False
        while True:
            scsPresentPosition, scsPresentSpeed = self.getPositionSpeed()
            print(f"[ID:{self.id:03d}] GoalPos:{goalPosition} PresPos:{scsPresentPosition} PresSpd:{scsPresentSpeed}")

            moving = self.isMoving()
            if moving:
                hasMoved = True

            if not moving and (hasMoved or abs(scsPresentPosition - goalPosition) <= self.positionTolerance):
                break

    #-----------------------------------------------------APPLY POSITION LIMITS-----------------------------------------------------------
    # Writes the hardware Min/Max_Angle_Limit registers from minPositionLimit/maxPositionLimit,
    # the same pattern every existing test script follows by hand around an EEPROM unlock/lock pair.

    def applyPositionLimits(self):
        self.unlockEeprom()
        self.setSetting(SMS_STS_MIN_ANGLE_LIMIT_L, self.minPositionLimit, 2, "Min Angle Limit", signed=True)
        self.setSetting(SMS_STS_MAX_ANGLE_LIMIT_L, self.maxPositionLimit, 2, "Max Angle Limit", signed=True)
        self.lockEeprom()

    #-----------------------------------------------------APPLY ANGLE RESOLUTION-----------------------------------------------------------
    # Writes a non-default Angle Resolution (EEPROM address 30) and fixes up Phase (address 18,
    # bit 4) so Present Position feedback reflects the larger angle instead of wrapping every
    # revolution - same pattern as applyPositionLimits(), demonstrated procedurally in main.py.

    def applyAngleResolution(self):
        self.unlockEeprom()
        self.setSetting(SMS_STS_ANGLE_RESOLUTION, self.angleResolution, 1, "Angle Resolution")
        phase = self.getSetting(SMS_STS_PHASE, 1, "Phase (before)")
        self.setSetting(SMS_STS_PHASE, phase | 0x10, 1, "Phase (multi-turn bit4 set)")
        self.lockEeprom()

    #-----------------------------------------------------PRINT METHOD-----------------------------------------------------------

    def printAllSettings(self):
        print(f"--- Motor ID {self.id} ---")
        print(f"  Raw Center Position: {self.rawCenterPosition}")
        print(f"  Range: {self.range}")
        print(f"  Min Position: {self.minPosition}")
        print(f"  Max Position: {self.maxPosition}")
        print(f"  Speed: {self.speed}")
        print(f"  Acceleration: {self.acceleration}")
        print(f"  Position Tolerance: {self.positionTolerance}")
        print(f"  Angle Resolution: {self.angleResolution}")
        print(f"  Min Position Limit: {self.minPositionLimit}")
        print(f"  Max Position Limit: {self.maxPositionLimit}")
        print(f"  Position Margin: {self.positionMargin}")
        print(f"  Present Position: {self.getPositionSpeed()[0]}")
        print("")
