
#-----------------------------------------------------GENERAL IMPORTS----------------------------------------------------------

import sys
import os
sys.path.append("..")
from scservo_sdk import *                      # Uses SC Servo SDK library
from motor import Motor

#-------------------------------------------------------SETTINGS BASED ON OS---------------------------------------------------

if os.name == 'nt':                             # 'nt' means Windows
    import msvcrt
    def getch():
        return msvcrt.getch().decode()

else:
    import tty, termios
    def getch():
        fd = sys.stdin.fileno()
        oldSettings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, oldSettings)
        return ch

#-----------------------------------------------------------BUS CLASS---------------------------------------------------------

class Bus:

    def __init__(self, deviceName, baudrate):

        self.deviceName    = deviceName                  # Serial port path (e.g. '/dev/tty.usbmodem5AAF2197651', 'COM1')
        self.baudrate      = baudrate                     # Bus baudrate (e.g. 1000000 for 1 Mbps)
        self.portHandler   = PortHandler(deviceName)       # PortHandlerLinux/PortHandlerWindows instance
        self.packetHandler = sms_sts(self.portHandler)      # sms_sts protocol instance shared by every motor on this bus

    #-----------------------------------------------------PORT LIFECYCLE METHODS-----------------------------------------------------------

    def openPort(self):
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

        if self.portHandler.setBaudRate(self.baudrate):
            print(f"Succeeded to change the baudrate to {self.baudrate}")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

    def closePort(self):
        self.portHandler.closePort()
        print("Port Closed")

    def __enter__(self):
        self.openPort()
        return self

    def __exit__(self, excType, excValue, traceback):
        self.closePort()

    #-----------------------------------------------------MOTOR FACTORY METHOD-----------------------------------------------------------
    # Convenience wrapper so callers don't have to pass this bus's packetHandler into every Motor by hand.

    def addMotor(self, id, rawCenterPosition, range, minPosition, maxPosition,
                 positionTolerance,
                 minPositionLimit, maxPositionLimit, positionMargin, angleResolution=1):
        return Motor(self.packetHandler, id, rawCenterPosition, range, minPosition, maxPosition,
                     positionTolerance,
                     minPositionLimit, maxPositionLimit, positionMargin, angleResolution=angleResolution)

    #-----------------------------------------------------SYNCHRONIZED MULTI-MOTOR MOVE-----------------------------------------------------------
    # Queues every motor's goal position/speed/acc, sends them all in one bus transaction so they
    # start moving together, then round-robin polls each motor until it has arrived.

    def moveMotorsToPositions(self, motors, moves):
        # motors: dict {id: Motor}
        # moves: dict {id: (goalPosition, speed, acceleration)}
        for id, (goalPosition, speed, acceleration) in moves.items():
            self.packetHandler.SyncWritePosEx(id, goalPosition, speed, acceleration)

        scsCommResult = self.packetHandler.groupSyncWrite.txPacket()
        if scsCommResult != COMM_SUCCESS:
            print(f"[SyncWrite] {self.packetHandler.getTxRxResult(scsCommResult)}")
        self.packetHandler.groupSyncWrite.clearParam()

        hasMoved = {id: False for id in moves}
        arrived = {id: False for id in moves}

        while not all(arrived.values()):
            for id, (goalPosition, speed, acceleration) in moves.items():
                if arrived[id]:
                    continue

                motor = motors[id]
                scsPresentPosition, scsPresentSpeed = motor.getPositionSpeed()
                print(f"[ID:{id:03d}] GoalPos:{goalPosition} PresPos:{scsPresentPosition} PresSpd:{scsPresentSpeed}")

                moving = motor.isMoving()
                if moving:
                    hasMoved[id] = True

                if not moving and (hasMoved[id] or abs(scsPresentPosition - goalPosition) <= motor.positionTolerance):
                    arrived[id] = True

    #-----------------------------------------------------PRINT METHOD-----------------------------------------------------------

    def printAllSettings(self):
        print(f"--- Bus ---")
        print(f"  Device Name: {self.deviceName}")
        print(f"  Baudrate: {self.baudrate}")
