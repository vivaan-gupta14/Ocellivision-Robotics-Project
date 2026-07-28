#--------------------------------------GENERAL IMPORTS----------------------------------------------------------

import sys
import os
import time
import select
print("GeneralImports Complete")

#---------------------------------------SETTINGS BASED ON OS---------------------------------------------------

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
print("OS Based Settings Complete")
#------------------------------------SERVO PYTHON LIBRARY IMPORT--------------------------------------------

sys.path.append("..")
from scservo_sdk import *                      # Uses SC Servo SDK library
print("SC Servo Library Imported")

#-------------------------------------DEFAULT SERVO SETTINGS-------------------------------------------------

# Default setting
BAUDRATE                    = 1000000           # SC Servo default baudrate : 1000000 (1 Mbps)
DEVICENAME                  = '/dev/tty.usbmodem5AAF2197651'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
SCS_MOVING_SPEED            = 2400           # SC Servo wheel-mode speed (slow, out of ~2400 used for full-speed moves)
SCS_MOVING_ACC              = 50            # SC Servo moving acc (50 steps/sec^2)

# The bus is daisy-chained (controller -> servo A -> servo B -> servo C -> servo D -> servo E), so
# all five servos share the same DEVICENAME/port and are addressed individually by their SCS ID.
# Each ID below must already be unique on the bus (use change_id.py beforehand - factory default is
# 1 for every servo, so plugging in several at once with no reassignment will make them collide).
# Each entry is (SCS ID, key that spins it negative/"left", key that spins it positive/"right").
SERVOS = [
    {'id': 6,  'neg_key': '1', 'pos_key': '2'},
    {'id': 7,  'neg_key': '3', 'pos_key': '4'},
    {'id': 8,  'neg_key': '5', 'pos_key': '6'},
    {'id': 9,  'neg_key': '7', 'pos_key': '8'},
    {'id': 10, 'neg_key': '9', 'pos_key': '0'},
]
print("Default Servo Settings Initialized")

#--------------------------------PORT AND PACKET HANDLER INITIALIZATION-------------------------------------

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
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

#--------------------------------------YOUR CODE HERE-----------------------------------------------------------

# A terminal only reports key-presses, never key-releases, so "holding" a key is detected
# via the OS's keyboard auto-repeat: while physically held, the same key keeps arriving
# every ~30-50ms. If no repeat arrives within RELEASE_TIMEOUT, that key is considered
# released and its servo is stopped. Each of the 4 servos tracks its own hold state
# independently, so any combination of the 4 key-pairs can be held at once.
RELEASE_TIMEOUT = 0.15   # seconds; may need tuning to your OS's key-repeat rate
POLL_INTERVAL   = 0.02   # seconds between non-blocking stdin polls

ESCAPE_SEQUENCE_TIMEOUT = 0.1  # time to wait for the rest of an arrow-key escape sequence

def read_key(timeout):
    """Non-blocking read of a single key event from stdin.
    Returns 'LEFT', 'RIGHT', 'ESC', a plain character (e.g. 'a', ',', '/'),
    or None if nothing usable arrived within timeout.

    Reads directly from the raw file descriptor (os.read) rather than the
    buffered sys.stdin: a buffered read can silently slurp a whole arrow-key
    escape sequence into its internal buffer on the first call, leaving
    nothing at the OS level for the next select() to see - making a real
    arrow press look like a lone ESC.
    """
    ready, _, _ = select.select([fd], [], [], timeout)
    if not ready:
        return None
    ch = os.read(fd, 1).decode()
    if ch != '\x1b':
        return ch
    # Possible arrow-key escape sequence: ESC [ C/D  or  ESC O C/D
    ready, _, _ = select.select([fd], [], [], ESCAPE_SEQUENCE_TIMEOUT)
    if not ready:
        return 'ESC'
    ch2 = os.read(fd, 1).decode()
    if ch2 not in ('[', 'O'):
        return None
    ready, _, _ = select.select([fd], [], [], ESCAPE_SEQUENCE_TIMEOUT)
    if not ready:
        return None
    ch3 = os.read(fd, 1).decode()
    if ch3 == 'C':
        return 'RIGHT'
    elif ch3 == 'D':
        return 'LEFT'
    return None

# Build a lookup from key -> (servo index, direction sign) from SERVOS above.
KEY_BINDINGS = {}
for _i, _servo in enumerate(SERVOS):
    KEY_BINDINGS[_servo['neg_key']] = (_i, -1)
    KEY_BINDINGS[_servo['pos_key']] = (_i, +1)

def stop_servo(i):
    scs_comm_result, scs_error = packetHandler.WriteSpec(SERVOS[i]['id'], 0, SCS_MOVING_ACC)
    if scs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(scs_comm_result))
    elif scs_error != 0:
        print("%s" % packetHandler.getRxPacketError(scs_error))

try:
    print("Starting the script...")

    if os.name == 'nt':
        raise RuntimeError("This arrow-key hold-control script requires Mac/Linux (uses select() on stdin).")

    # Put all 4 servos into continuous-rotation (wheel) mode
    for servo in SERVOS:
        scs_comm_result, scs_error = packetHandler.WheelMode(servo['id'])
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % packetHandler.getRxPacketError(scs_error))

    print("Hold %s to rotate servo %s, press ESC to quit."
          % (', '.join("%s/%s" % (s['neg_key'], s['pos_key']) for s in SERVOS),
             ', '.join(str(s['id']) for s in SERVOS)))

    tty.setraw(fd)  # raw mode so arrow-key escape sequences arrive unbuffered

    current_sign = [0] * len(SERVOS)     # 0 = stopped, -1/+1 = current spin direction
    last_key_time = [0.0] * len(SERVOS)

    while True:
        key = read_key(POLL_INTERVAL)
        now = time.time()

        if key == 'ESC':
            break
        elif key in KEY_BINDINGS:
            idx, sign = KEY_BINDINGS[key]
            last_key_time[idx] = now
            if sign != current_sign[idx]:
                current_sign[idx] = sign
                speed = sign * SCS_MOVING_SPEED
                scs_comm_result, scs_error = packetHandler.WriteSpec(SERVOS[idx]['id'], speed, SCS_MOVING_ACC)
                if scs_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(scs_comm_result))
                elif scs_error != 0:
                    print("%s" % packetHandler.getRxPacketError(scs_error))

        # Independently of whatever key (if any) just arrived, check every servo for release:
        # each one stops itself if its own key hasn't repeated within RELEASE_TIMEOUT.
        for i in range(len(SERVOS)):
            if current_sign[i] != 0 and (now - last_key_time[i]) > RELEASE_TIMEOUT:
                current_sign[i] = 0
                stop_servo(i)

#--------------------------------------ENDING THE SCRIPT-------------------------------------------------------

finally:
    # Make sure every motor is stopped before closing
    for i in range(len(SERVOS)):
        stop_servo(i)
    if os.name != 'nt':
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    # Close port
    portHandler.closePort()
    print("Port Closed")
