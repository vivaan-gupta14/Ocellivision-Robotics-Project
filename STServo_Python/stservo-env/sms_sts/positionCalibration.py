
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

#---------------------------------------------------GET AND SET SETTINGS FUNCTIONS-----------------------------------------------------------

def setSetting(scs_id, address, value, num_bytes, name):
    if num_bytes == 1:
        scs_comm_result, scs_error = packetHandler.write1ByteTxRx(scs_id, address, value)
    else:
        scs_comm_result, scs_error = packetHandler.write2ByteTxRx(scs_id, address, value)

    if scs_comm_result != COMM_SUCCESS:
        print(f"[ID:{scs_id:03d}][{name}] {packetHandler.getTxRxResult(scs_comm_result)}")
    elif scs_error != 0:
        print(f"[ID:{scs_id:03d}][{name}] {packetHandler.getRxPacketError(scs_error)}")
    else:
        print(f"[ID:{scs_id:03d}][{name}] Set to {value}")

def getSetting(scs_id, address, num_bytes, name):
    if num_bytes == 1:
        value, scs_comm_result, scs_error = packetHandler.read1ByteTxRx(scs_id, address)
    else:
        value, scs_comm_result, scs_error = packetHandler.read2ByteTxRx(scs_id, address)

    if scs_comm_result != COMM_SUCCESS:
        print(f"[ID:{scs_id:03d}][{name}] {packetHandler.getTxRxResult(scs_comm_result)}")
    elif scs_error != 0:
        print(f"[ID:{scs_id:03d}][{name}] {packetHandler.getRxPacketError(scs_error)}")
    else:
        print(f"[ID:{scs_id:03d}][{name}] Read {value}")
    return value

#-----------------------------------------------------MOVE TO POSITION FUNCTIONS-----------------------------------------------------------

def move_to_position(packetHandler, scs_id, goal_position, moving_speed, moving_acc, tolerance):
    # Write SC Servo goal position/moving speed/moving acc
    scs_comm_result, scs_error = packetHandler.WritePosEx(scs_id, goal_position, moving_speed, moving_acc)
    if scs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(scs_comm_result))
    elif scs_error != 0:
        print("%s" % packetHandler.getRxPacketError(scs_error))

    has_moved = False
    while 1:
        # Read SC Servo present position
        scs_present_position, scs_present_speed, scs_comm_result, scs_error = packetHandler.ReadPosSpeed(scs_id)
        if scs_comm_result != COMM_SUCCESS:
            print(packetHandler.getTxRxResult(scs_comm_result))
        else:
            print("[ID:%03d] GoalPos:%d PresPos:%d PresSpd:%d" % (scs_id, goal_position, scs_present_position, scs_present_speed))
        if scs_error != 0:
            print(packetHandler.getRxPacketError(scs_error))

        # Read SC Servo moving status
        moving, scs_comm_result, scs_error = packetHandler.ReadMoving(scs_id)
        if scs_comm_result != COMM_SUCCESS:
            print(packetHandler.getTxRxResult(scs_comm_result))

        if moving == 1:
            has_moved = True

        # Trust moving==0 as "arrived" once we've seen it move, OR if we're already
        # within the servo's dead zone (it may never set moving==1 if the goal is
        # close enough that it decides no move is needed).
        if moving == 0 and (has_moved or abs(scs_present_position - goal_position) <= tolerance):
            break

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

# Per-motor configuration. Fill in the desired center rod position, range,
# moving speed, and moving acc for each of the four motors before running.
MOTORS = {
    7:  {"desired_center_raw": 2750, "range": 600, "moving_speed": 1200, "moving_acc": 10, "tolerance": 3},
    8:  {"desired_center_raw": 1750, "range": 700, "moving_speed": 1200, "moving_acc": 10, "tolerance": 3},
    9:  {"desired_center_raw": 3385, "range": 800, "moving_speed": 1200, "moving_acc": 10, "tolerance": 3},
    10: {"desired_center_raw": 3860, "range": 800, "moving_speed": 1200, "moving_acc": 10, "tolerance": 3},
}

# Derive per-motor correction/min/max from the config above
for scs_id, cfg in MOTORS.items():
    cfg["correction"]    = cfg["desired_center_raw"] - SCS_CALIBRATED_CENTER
    cfg["min_position"]  = SCS_CALIBRATED_CENTER - cfg["range"]
    cfg["max_position"]  = SCS_CALIBRATED_CENTER + cfg["range"]

print(f"SC Servo BAUDRATE: {BAUDRATE}")
print(f"SC Servo DEVICENAME: {DEVICENAME}")
print(f"SC Servo Calibrated Center: {SCS_CALIBRATED_CENTER}")
for scs_id, cfg in MOTORS.items():
    print(f"--- Motor ID {scs_id} ---")
    print(f"  Desired Center (Raw): {cfg['desired_center_raw']}")
    print(f"  Range From Center: {cfg['range']}")
    print(f"  Moving Speed: {cfg['moving_speed']}")
    print(f"  Moving Acc: {cfg['moving_acc']}")
    print(f"  Position Tolerance: {cfg['tolerance']}")
    print(f"  Correction: {cfg['correction']}")
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

#------------------------------------------------------------POSITION CORRECTION----------------------------------------------

# Move all motors to their desired physical orientation first, simultaneously
print("Moving all motors to their target physical center angle...")
move_all_to_positions(packetHandler, MOTORS, {scs_id: cfg["desired_center_raw"] for scs_id, cfg in MOTORS.items()})
time.sleep(2.0) # Ensure they have completely stopped vibrating

print("--- Starting Offset Calibration Process ---")

for scs_id, cfg in MOTORS.items():
    print(f"-- Motor ID {scs_id} --")

    # Step 1: Unlock EEPROM Configuration Memory (Address 55 = 0)
    setSetting(scs_id, address=55, value=0, num_bytes=1, name="EEPROM Lock")
    time.sleep(0.1)

    # Step 2: Write the offset delta to Register 31 (2 Bytes)
    setSetting(scs_id, address=31, value=cfg["correction"], num_bytes=2, name="Position Offset (OFS)")
    time.sleep(0.1)

    # Step 3: Lock EEPROM back down (Address 55 = 1) to protect the memory
    setSetting(scs_id, address=55, value=1, num_bytes=1, name="EEPROM Lock")
    time.sleep(0.2)

    # Step 4: Verify the offset was written successfully
    getSetting(scs_id, address=31, num_bytes=2, name="Verified Offset")

# Step 5: Move all motors to the newly calibrated center and confirm they read 2048
print("Moving all motors to calibrated center...")
move_all_to_positions(packetHandler, MOTORS, {scs_id: SCS_CALIBRATED_CENTER for scs_id in MOTORS})

for scs_id in MOTORS:
    scs_present_position, _, _, _ = packetHandler.ReadPosSpeed(scs_id)
    print(f"[ID:{scs_id:03d}] Present position after calibration: {scs_present_position}")

print(f"--- Calibration Complete --- \n")
time.sleep(2)

#------------------------------------------------------------RANGE OF MOTION CHECK--------------------------------------------

print("--- Verifying Range of Motion ---")

print("Moving all motors to their minimum position...")
move_all_to_positions(packetHandler, MOTORS, {scs_id: cfg["min_position"] for scs_id, cfg in MOTORS.items()})
time.sleep(2.0)

print("Moving all motors to calibrated center...")
move_all_to_positions(packetHandler, MOTORS, {scs_id: SCS_CALIBRATED_CENTER for scs_id in MOTORS})
time.sleep(2.0)

print("Moving all motors to their maximum position...")
move_all_to_positions(packetHandler, MOTORS, {scs_id: cfg["max_position"] for scs_id, cfg in MOTORS.items()})
time.sleep(2.0)

print("Moving all motors to calibrated center...")
move_all_to_positions(packetHandler, MOTORS, {scs_id: SCS_CALIBRATED_CENTER for scs_id in MOTORS})
time.sleep(2.0)

print("--- Range of Motion Check Complete --- \n")

portHandler.closePort()
print("Port Closed")
