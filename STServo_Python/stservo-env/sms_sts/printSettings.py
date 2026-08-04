
#-----------------------------------------------------GENERAL IMPORTS----------------------------------------------------------

import sys
sys.path.append("..")
from scservo_sdk import *                      # Uses SC Servo SDK library
from bus import Bus

#-------------------------------------------------------INITIALIZE SERVO SETTINGS-------------------------------------------------

SCS_ID                          = 6                                 # SC Servo ID to read settings from
BAUDRATE                        = 1000000                           # SC Servo default baudrate : 1000000 (1 Mbps)
DEVICENAME                      = '/dev/tty.usbmodem5AAF2197651'    # Check which port is being used on your controller
                                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#-------------------------------------------------------FULL CONTROL TABLE-------------------------------------------------------
# (address, num_bytes, name, signed) for every register in the STS/SCS control table.
# signed=True registers use the sign-magnitude encoding (bit15 = sign) that Motor.getSetting
# already decodes via scs_tohost - the same convention used for Present_Position elsewhere
# in this project. EEPROM registers (address < 40) persist across power cycles; SRAM
# registers (address >= 40) reset to their defaults on power-up.

EEPROM_TABLE = [
    (0,  1, "Firmware Major Version",              False),
    (1,  1, "Firmware Minor Version",              False),
    (3,  2, "Model Number",                        False),
    (5,  1, "ID",                                  False),
    (6,  1, "Baud Rate",                           False),
    (7,  1, "Return Delay Time",                   False),
    (8,  1, "Response Status Level",               False),
    (9,  2, "Min Angle Limit",                     True),
    (11, 2, "Max Angle Limit",                     True),
    (13, 1, "Max Temperature Limit",               False),
    (14, 1, "Max Input Voltage",                   False),
    (15, 1, "Min Input Voltage",                   False),
    (16, 2, "Max Torque",                          False),
    (18, 1, "Phase (Setting Byte)",                False),
    (19, 1, "Unloading Condition",                 False),
    (20, 1, "LED Alarm Condition",                 False),
    (21, 1, "P Proportional Coefficient",          False),
    (22, 1, "D Differential Coefficient",          False),
    (23, 1, "I Integral Coefficient",              False),
    (24, 2, "Minimum Startup Force",               False),
    (26, 1, "CW Dead Zone",                        False),
    (27, 1, "CCW Dead Zone",                       False),
    (28, 2, "Protection Current",                  False),
    (30, 1, "Angular Resolution",                  False),
    (31, 2, "Position Correction (Offset)",        True),
    (33, 1, "Operation Mode",                      False),
    (34, 1, "Protective Torque",                   False),
    (35, 1, "Protection Time",                     False),
    (36, 1, "Overload Torque",                     False),
    (37, 1, "Speed Closed-Loop P Coefficient",     False),
    (38, 1, "Over Current Protection Time",        False),
    (39, 1, "Velocity Closed-Loop I Coefficient",  False),
]

SRAM_TABLE = [
    (40, 1, "Torque Enable",               False),
    (41, 1, "Acceleration",                False),
    (42, 2, "Goal Position",               True),
    (44, 2, "Goal Time (Running Time)",    False),
    (46, 2, "Goal Speed",                  True),
    (48, 2, "Torque Limit",                False),
    (55, 1, "Lock",                        False),
    (56, 2, "Present Position",            True),
    (58, 2, "Present Speed",               True),
    (60, 2, "Present Load",                False),
    (62, 1, "Present Voltage",             False),
    (63, 1, "Present Temperature",         False),
    (64, 1, "Async Write Status",          False),
    (65, 1, "Servo Status",                False),
    (66, 1, "Moving",                      False),
    (69, 2, "Present Current",             True),
]

#--------------------------------------------------BUS AND MOTOR INITIALIZATION-------------------------------------

with Bus(DEVICENAME, BAUDRATE) as bus:

    # Only used for reading here, so the move-related fields (speed, acceleration, limits, ...)
    # are irrelevant placeholders - Motor.getSetting doesn't touch them.
    motor = bus.addMotor(
        id=SCS_ID,
        rawCenterPosition=0,
        range=0,
        minPosition=0,
        maxPosition=0,
        speed=0,
        acceleration=0,
        positionTolerance=0,
        angleResolution=0,
        minPositionLimit=0,
        maxPositionLimit=0,
        positionMargin=0,
    )

    print(f"\n=== EEPROM Settings (persist across power cycles) - ID {SCS_ID} ===")
    for address, numBytes, name, signed in EEPROM_TABLE:
        motor.getSetting(address, numBytes, name, signed)

    print(f"\n=== SRAM Settings (reset on power-up) - ID {SCS_ID} ===")
    for address, numBytes, name, signed in SRAM_TABLE:
        motor.getSetting(address, numBytes, name, signed)
