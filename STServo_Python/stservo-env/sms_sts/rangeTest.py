
#-----------------------------------------------------GENERAL IMPORTS----------------------------------------------------------

import sys
import os
import time
print("General Imports Complete\n")

#-------------------------------------------------------SETTINGS BASED ON OS---------------------------------------------------

if os.name == 'nt': #'nt' means Windows
    print("OS detected as Windows")
    import msvcrt
    def getch():
        return msvcrt.getch().decode()

else:
    print("OS detected as Linux/Mac")
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
print("OS Based Settings Complete\n")


#-------------------------------------------------------SERVO PYTHON LIBRARY IMPORT--------------------------------------------

sys.path.append("..")
from scservo_sdk import *                      # Uses SC Servo SDK library
print("SC Servo Library Imported\n")

#-----------------------------------------------------MOVE TO POSITION FUNCTIONS-----------------------------------------------------------

def move_all_to_positions(packetHandler, motors, goal_positions):
    # goal_positions: {scs_id: goal_position}
    # motors: {scs_id: {"moving_speed":..., "moving_acc":..., "tolerance":...}}

    # Queue every servo's goal position/speed/acc, then send them all in one
    # bus transaction so the motors start moving together instead of in sequence.
    for scs_id, goal_position in goal_positions.items():
        cfg = motors[scs_id]
        packetHandler.SyncWritePosEx(scs_id, goal_position, cfg["moving_speed"], cfg["moving_acc"])

    scs_comm_result = packetHandler.groupSyncWrite.txPacket()
    if scs_comm_result != COMM_SUCCESS:
        print("[SyncWrite] %s" % packetHandler.getTxRxResult(scs_comm_result))
    packetHandler.groupSyncWrite.clearParam()

    # Poll every servo in a round-robin until all of them have arrived.
    has_moved = {scs_id: False for scs_id in goal_positions}
    arrived = {scs_id: False for scs_id in goal_positions}

    while not all(arrived.values()):
        for scs_id, goal_position in goal_positions.items():
            if arrived[scs_id]:
                continue

            scs_present_position, scs_present_speed, scs_comm_result, scs_error = packetHandler.ReadPosSpeed(scs_id)
            if scs_comm_result != COMM_SUCCESS:
                print(packetHandler.getTxRxResult(scs_comm_result))
                continue
            else:
                print("[ID:%03d] GoalPos:%d PresPos:%d PresSpd:%d" % (scs_id, goal_position, scs_present_position, scs_present_speed))
            if scs_error != 0:
                print(packetHandler.getRxPacketError(scs_error))

            moving, scs_comm_result, scs_error = packetHandler.ReadMoving(scs_id)
            if scs_comm_result != COMM_SUCCESS:
                print(packetHandler.getTxRxResult(scs_comm_result))
                continue

            if moving == 1:
                has_moved[scs_id] = True

            tolerance = motors[scs_id]["tolerance"]
            if moving == 0 and (has_moved[scs_id] or abs(scs_present_position - goal_position) <= tolerance):
                arrived[scs_id] = True

#-------------------------------------------------------INITIALIZE SERVO SETTINGS-------------------------------------------------

BAUDRATE                        = 1000000                           # SC Servo default baudrate : 1000000 (1 Mbps)
DEVICENAME                      = '/dev/tty.usbmodem5AAF2197651'    # Check which port is being used on your controller
                                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

SCS_CALIBRATED_CENTER           = 2048                              # The position value that becomes the "center" position after correction

# Per-motor configuration. This assumes motors have already been offset-calibrated
# (via positionCalibration.py) so that SCS_CALIBRATED_CENTER is their true center.
MOTORS = {
    7:  {"range": 700, "moving_speed": 1200, "moving_acc": 10, "tolerance": 3},
    8:  {"range": 700, "moving_speed": 1200, "moving_acc": 10, "tolerance": 3},
    9:  {"range": 1400, "moving_speed": 1200, "moving_acc": 10, "tolerance": 3},
    10: {"range": 1400, "moving_speed": 1200, "moving_acc": 10, "tolerance": 3},
}

# Derive per-motor min/max from the config above
for scs_id, cfg in MOTORS.items():
    cfg["min_position"]  = SCS_CALIBRATED_CENTER - cfg["range"]
    cfg["max_position"]  = SCS_CALIBRATED_CENTER + cfg["range"]

print(f"SC Servo BAUDRATE: {BAUDRATE}")
print(f"SC Servo DEVICENAME: {DEVICENAME}")
print(f"SC Servo Calibrated Center: {SCS_CALIBRATED_CENTER}")
for scs_id, cfg in MOTORS.items():
    print(f"--- Motor ID {scs_id} ---")
    print(f"  Range From Center: {cfg['range']}")
    print(f"  Moving Speed: {cfg['moving_speed']}")
    print(f"  Moving Acc: {cfg['moving_acc']}")
    print(f"  Position Tolerance: {cfg['tolerance']}")
    print(f"  Minimum Position: {cfg['min_position']}")
    print(f"  Maximum Position: {cfg['max_position']}")
print("")

#--------------------------------------------------PORT AND PACKET HANDLER INITIALIZATION-------------------------------------

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)
print("Port Handler Initialized")

# Initialize PacketHandler instance
# Get methods and members of Protocol
#Converts code instructions into bytes
packetHandler = sms_sts(portHandler)
print("Packet Handler Initialized")


# Open port with error checking
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate with error checking
if portHandler.setBaudRate(BAUDRATE):
    print(f"Succeeded to change the baudrate to {BAUDRATE}")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

print("")

#------------------------------------------------------------RANGE OF MOTION CHECK--------------------------------------------

print("--- Verifying Range of Motion ---")

for scs_id, cfg in MOTORS.items():
    print(f"--- Motor ID {scs_id} ---")

    print(f"Moving motor {scs_id} to its minimum position...")
    move_all_to_positions(packetHandler, MOTORS, {scs_id: cfg["min_position"]})
    time.sleep(2.0)

    print(f"Moving motor {scs_id} to calibrated center...")
    move_all_to_positions(packetHandler, MOTORS, {scs_id: SCS_CALIBRATED_CENTER})
    time.sleep(2.0)

    print(f"Moving motor {scs_id} to its maximum position...")
    move_all_to_positions(packetHandler, MOTORS, {scs_id: cfg["max_position"]})
    time.sleep(2.0)

    print(f"Moving motor {scs_id} to calibrated center...")
    move_all_to_positions(packetHandler, MOTORS, {scs_id: SCS_CALIBRATED_CENTER})
    time.sleep(2.0)

print("--- Range of Motion Check Complete --- \n")

portHandler.closePort()
print("Port Closed")
