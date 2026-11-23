

#uv run simHardware.py --follower-port /dev/ttyACM0 --follower-id follower_arm

#------------------------------------------------------IMPORTS------------------------------------------------------#

import time
import numpy as np
import math
import threading

#PyBullet Imports
import pybullet as p
import pybullet_data

#Pygame Imports (for PS4 Controller)
import pygame

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode
from lerobot.robots.so_follower import SOFollowerRobotConfig
from so101_follower_5dof import SO101Follower5DOF


print("Imports completed")

#----------------------------------------------------HARDWARE CONSTANTS----------------------------------------------------
# Copied exactly from 07_teleoperate_full_10dof.py -- do not retune these here, this file
# is not the source of truth for them.

ARM_MOTOR_NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
# This URDF's joint names for the same 5 joints, same position in the chain -- "wrist_to_rail"
# vs "wrist_roll" naming mismatch is real, not a typo. Sign correction below is confirmed on
# hardware (see 07_teleoperate_full_10dof.py's originating script).
PYBULLET_ARM_JOINT_NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_to_rail")
ARM_JOINT_SIGNS = {
    "shoulder_pan": 1,
    "shoulder_lift": 1,
    "elbow_flex": 1,
    "wrist_flex": 1,
    "wrist_roll": -1,
}

# ID 7-10 real joint mapping, as given: 7=wrist_pitch, 8=roll, 9=jaw_1, 10=jaw_2
DISK_MOTOR_NAMES = {7: "wrist_pitch", 8: "roll", 9: "jaw_1", 10: "jaw_2"}

# --- Protection/alarm/dead-zone -- copied exactly from 07_teleoperate_full_10dof.py ---
MAX_TEMPERATURE_LIMIT = 55
MAX_VOLTAGE_LIMIT, MIN_VOLTAGE_LIMIT = 130, 90
UNLOADING_CONDITIONS, LED_ALARM_CONDITIONS = 63, 63
CW_INSENSITIVE_ZONE, CCW_INSENSITIVE_ZONE = 1, 1

# --- Motors 7-10 config -- copied exactly from 07_teleoperate_full_10dof.py ---
# ROTARY_TORQUE_LIMIT = 125 is in 0.1% units -> 12.5% of max torque. Matches diskTorqueNM's
# 12.5% figure already used in this sim's own SET JOINT PROPERTIES section -- consistent
# with what's already established, not a new number.
SCS_CALIBRATED_CENTER = 2048
ROTARY_TORQUE_LIMIT = 125
MOTOR_CONFIGS = {
    7:  {"range": 700,  "moving_speed": 1200, "moving_acc": 10, "margin": 100},
    8:  {"range": 700,  "moving_speed": 1200, "moving_acc": 10, "margin": 100},
    9:  {"range": 1400, "moving_speed": 1200, "moving_acc": 10, "margin": 100},
    10: {"range": 1400, "moving_speed": 1200, "moving_acc": 10, "margin": 100},
}

# UNVERIFIED, unlike ARM_JOINT_SIGNS above. Sim-positive direction for these 4 joints has
# never been checked against real servo direction. Before trusting these for anything beyond
# a slow, small, watched test move: command +1 tick on each individually and confirm by eye
# it matches which way the sim joint rotates for a positive value.
DISK_JOINT_SIGNS = {"roll": 1, "wrist_pitch": 1, "jaw_1": 1, "jaw_2": 1}
TICKS_PER_RADIAN = 4096 / (2 * math.pi)



#-------------------------------------------------THREADING FUNCTIONS------------------------------------------------------#

#Get's the PS4 controller data and updates variable dictionary using lock
def getPS4Data(lock, variables, stopEvent, controller):
    DIVIDING_FACTOR = 1000
    L2Trigger = 5
    R2Trigger = 2
    while not stopEvent.is_set():
        pygame.event.pump()
        with lock:
            if round(controller.get_axis(L2Trigger), 1) != 0.0:
                variables["z"] = round(controller.get_axis(L2Trigger), 1)/DIVIDING_FACTOR
            elif round(controller.get_axis(R2Trigger), 1) != 0.0:
                variables["z"] = round(controller.get_axis(R2Trigger), 1)/DIVIDING_FACTOR
            else:
                variables["z"] = 0.0
            
        time.sleep(0.01)

def getPygameData(lock, variables, stopEvent):
    STEP = 0.001
    while not stopEvent.is_set():
        pygame.event.pump()
        keys = pygame.key.get_pressed()
        with lock:
            if keys[pygame.K_PAGEDOWN]:
                variables["z"] = STEP
            elif keys[pygame.K_PAGEUP]:
                variables["z"] = -1*STEP
            else:
                variables["z"] = 0.0

            if keys[pygame.K_UP]:
                variables["x"] = STEP
            elif keys[pygame.K_DOWN]:
                variables["x"] = -1* STEP
            else:
                variables["x"] = 0.0

            if keys[pygame.K_LEFT]:
                variables["y"] = STEP
            elif keys[pygame.K_RIGHT]:
                variables["y"] = -1*STEP
            else:
                variables["y"] = 0.0

            if keys[pygame.K_e]:
                variables["roll"] = STEP
            elif keys[pygame.K_q]:
                variables["roll"] = -1*STEP
            else:
                variables["roll"] = 0.0            
            
            if keys[pygame.K_w]:
                variables["pitch"] = STEP
            elif keys[pygame.K_s]:
                variables["pitch"] = -1*STEP
            else:
                variables["pitch"] = 0.0

            if keys[pygame.K_d]:
                variables["yaw"] = STEP
            elif keys[pygame.K_a]:
                variables["yaw"] = -1*STEP
            else:
                variables["yaw"] = 0.0
        time.sleep(0.01)

#--------------------------------------------------HELPER FUNCTIONS------------------------------------------------------#

def getJointPositions(bodyUniqueId, movableJointIndices, printPositions=False):
    jointStates = p.getJointStates(bodyUniqueId, movableJointIndices)
    jointPositions = [state[0] for state in jointStates]
    if printPositions:
        print(f"\nJoint Positions: {dict(zip(movableJointNames, jointPositions))}\n")
    return jointPositions

def getSimTCPPose(bodyUniqueID, TCPIndex, printPose = False):
    linkState = p.getLinkState(bodyUniqueID, TCPIndex, computeForwardKinematics = 1)
    tcpPosition = linkState[4]
    tcpRotQuat = linkState[5]
    if printPose:
        print(f"\nTCP Position: {tcpPosition}\nTCP Rotation (Quaternion): {tcpRotQuat}\nTCP Rotation (Euler): {p.getEulerFromQuaternion(tcpRotQuat)}\n")
    return tcpPosition, tcpRotQuat

def moveRobot(robot, TCPIndex, allmovableJointIndices, minJointLimits, maxJointLimits, jointRanges, jointDamping, targetTCPPosition, targetTCPRotQuat):
    currentJointPositions = getJointPositions(robot, allmovableJointIndices)

    print("creating target joint positions...")
    targetJointPositions = p.calculateInverseKinematics(
        robot, TCPIndex, targetTCPPosition,targetOrientation = targetTCPRotQuat,
        lowerLimits = minJointLimits,
        upperLimits = maxJointLimits,
        jointRanges = jointRanges,
        restPoses = currentJointPositions,
        jointDamping = jointDamping,
        # solver = p.IK_SDLS,
        maxNumIterations = 500,
        residualThreshold = 1e-5,)

    #move to those positions
    for jointIndex, jointPosition in zip(allmovableJointIndices, targetJointPositions):
        p.resetJointState(robot, jointIndex, jointPosition)

    POSITION_TOLERANCE_M = 0.1
    ROTATION_TOLERANCE_DEGREES = 10
    FKPosition, FKRotQuat = getSimTCPPose(robot, TCPIndex)
    positionDistance = math.dist(FKPosition, targetTCPPosition)

    rotDotProduct = np.clip(abs(np.dot(targetTCPRotQuat, FKRotQuat)), -1.0, 1.0)
    rotErrorDegrees = np.degrees (2 * np.arccos(rotDotProduct))

    if positionDistance > POSITION_TOLERANCE_M:
        print(f"IK FAILED - TARGET POSITION IS NOT REACHABLE. Position Distance {positionDistance}")
        for jointIndex, jointPosition in zip(allmovableJointIndices, currentJointPositions):
            p.resetJointState(robot, jointIndex, jointPosition)
    elif rotErrorDegrees > ROTATION_TOLERANCE_DEGREES:
        print(f"IK FAILED - TARGET ROTATION IS NOT REACHABLE. Rotational Error (Degrees) {rotErrorDegrees}")
        for jointIndex, jointPosition in zip(allmovableJointIndices, currentJointPositions):
            p.resetJointState(robot, jointIndex, jointPosition)
    else:
        print("moving robot (sim)")
        p.setJointMotorControlArray(
            robot, allmovableJointIndices, p.POSITION_CONTROL,
            targetPositions=targetJointPositions,
            positionGains=[0.02]*len(allmovableJointIndices),
            velocityGains=[1.0]*len(allmovableJointIndices),
            forces=[500]*len(allmovableJointIndices))
        print("moving robot (hardware)")
        sendJointPositionsToRobot(bus, targetJointPositions, movableJointNames, config)

def sendJointPositionsToRobot(bus, targetJointPositions, movableJointNames, config):
    jointValueByName = dict(zip(movableJointNames, targetJointPositions))

    # Arm (motors 1-5): radians -> degrees, sign-corrected (confirmed on hardware), real motor names
    armGoalDegrees = {}
    for simName, motorName in zip(PYBULLET_ARM_JOINT_NAMES, ARM_MOTOR_NAMES):
        armGoalDegrees[motorName] = ARM_JOINT_SIGNS[motorName] * math.degrees(jointValueByName[simName])
    bus.sync_write("Goal_Position", armGoalDegrees)  # normalize=True (default), degrees per calibration

    # Disks (motors 7-10): radians -> raw ticks around calibrated center, normalize=False
    diskGoalTicks = {}
    for wid, motorName in DISK_MOTOR_NAMES.items():
        radiansValue = jointValueByName[motorName]
        signedTicks = DISK_JOINT_SIGNS[motorName] * round(radiansValue * TICKS_PER_RADIAN)
        diskGoalTicks[motorName] = SCS_CALIBRATED_CENTER + signedTicks
    bus.sync_write("Goal_Position", diskGoalTicks, normalize=False)

    # carriage_prismatic_joint's IK result is intentionally NOT sent -- motor 6 needs its own
    # sensorless-homing-aware integration (home_motor_6 in 07_teleoperate_full_10dof.py),
    # not something this function's pattern is safe to extend to.

#-------------------------------------------CREATING PYBULLET SIMULATION------------------------------------------------------#

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, 0.0)

planeID = p.loadURDF("plane.urdf")

robot = p.loadURDF("/home/user/Desktop/Vivaan Internship Project/Ocellivision-Robotics-Project/Robot Files/full_assembly.urdf", 
                   baseOrientation=p.getQuaternionFromEuler([0, 0, 0]), 
                   useFixedBase=1,
                   flags = p.URDF_USE_SELF_COLLISION)

p.setRealTimeSimulation(0)

#--------------------------------------------SET SIM JOINT PROPERTIES-----------------------------------------------------
    
allJointInfo = {}

for i in range(p.getNumJoints(robot)):
    jointInfo = p.getJointInfo(robot, i)
    jointName = jointInfo[1].decode("utf-8")
    jointType = jointInfo[2]
    jointLowerLimit = jointInfo[8]
    jointUpperLimit = jointInfo[9]
    allJointInfo[jointName] = {
        "index": i,
        "type": jointType,
        "lowerLimit": jointLowerLimit,
        "upperLimit": jointUpperLimit
    }
    childLinkName = jointInfo[12].decode('utf-8')
    if childLinkName == "PSM1_tool_tip_link":
        TCPIndex = i

print("Joints Found:", list(allJointInfo.keys()))

armJointNames = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_to_rail"]
prismaticJointName = "carriage_prismatic_joint"
daVinciJointNames = ["roll", "wrist_pitch", "jaw_1", "jaw_2"]

movableJointNames = armJointNames + [prismaticJointName] + daVinciJointNames
movableJointIndices = [allJointInfo[name]["index"] for name in movableJointNames]
movableJointLowerLimits = [allJointInfo[name]["lowerLimit"] for name in movableJointNames]
movableJointUpperLimits = [allJointInfo[name]["upperLimit"] for name in movableJointNames]
movableJointRanges = [movableJointUpperLimits[i] - movableJointLowerLimits[i] for i in range(len(movableJointIndices))]
movableJointDampings = []
for name in movableJointNames:
    movableJointDampings.append(1.0)

position, orientation = p.getBasePositionAndOrientation(robot)
print(f"Base Position: {position}")
print(f"Base Orientation (Quaternion): {orientation}")

#--------------------------------------------CONNECT TO REAL ARM HARDWARE----------------------------------------------------#

parser = argparse.ArgumentParser()
parser.add_argument("--follower-port", required=True)
parser.add_argument("--follower-id", required=True)
args = parser.parse_args()

calib_source = SO101Follower5DOF(SOFollowerRobotConfig(port=args.follower_port, id=args.follower_id))
calib_1_5 = calib_source.calibration
if not calib_1_5:
    raise RuntimeError(
        f"No calibration found for follower id '{args.follower_id}' -- run 05_calibrate_follower.py first."
    )
config = calib_source.config
del calib_source  # never call .connect() on it

motors = {name: Motor(i, "sts3215", MotorNormMode.DEGREES) for i, name in zip(range(1, 6), ARM_MOTOR_NAMES)}
for wid, wname in DISK_MOTOR_NAMES.items():
    # Placeholder norm_mode -- unused, every disk read/write always passes normalize=False.
    motors[wname] = Motor(wid, "sts3215", MotorNormMode.RANGE_M100_100)

bus = FeetechMotorsBus(port=args.follower_port, motors=motors, calibration=calib_1_5)
bus.connect()  # handshake: pings all 9 IDs (1-5, 7-10) + firmware check only
bus.write_calibration(calib_1_5, cache=True)
bus.configure_motors()

with bus.torque_disabled(list(ARM_MOTOR_NAMES)):
    for name in ARM_MOTOR_NAMES:
        bus.write("Operating_Mode", name, OperatingMode.POSITION.value)
        # bus.write("P_Coefficient", name, config.position_p_coefficient)
        # bus.write("I_Coefficient", name, config.position_i_coefficient)
        # bus.write("D_Coefficient", name, config.position_d_coefficient)

# --- THE PART YOU ASKED FOR: safety settings for motors 7-10 ---
with bus.torque_disabled(list(DISK_MOTOR_NAMES.values())):
    for wname in DISK_MOTOR_NAMES.values():
        bus.write("Max_Temperature_Limit", wname, MAX_TEMPERATURE_LIMIT, normalize=False)
        bus.write("Max_Voltage_Limit", wname, MAX_VOLTAGE_LIMIT, normalize=False)
        bus.write("Min_Voltage_Limit", wname, MIN_VOLTAGE_LIMIT, normalize=False)
        bus.write("Unloading_Condition", wname, UNLOADING_CONDITIONS, normalize=False)
        bus.write("LED_Alarm_Condition", wname, LED_ALARM_CONDITIONS, normalize=False)
        bus.write("CW_Dead_Zone", wname, CW_INSENSITIVE_ZONE, normalize=False)
        bus.write("CCW_Dead_Zone", wname, CCW_INSENSITIVE_ZONE, normalize=False)

with bus.torque_disabled(list(DISK_MOTOR_NAMES.values())):
    for wid, cfg in MOTOR_CONFIGS.items():
        wname = DISK_MOTOR_NAMES[wid]
        min_pos = SCS_CALIBRATED_CENTER - cfg["range"]
        max_pos = SCS_CALIBRATED_CENTER + cfg["range"]
        bus.write("Min_Position_Limit", wname, min_pos - cfg["margin"], normalize=False)
        bus.write("Max_Position_Limit", wname, max_pos + cfg["margin"], normalize=False)
        bus.write("Operating_Mode", wname, OperatingMode.POSITION.value, normalize=False)
        bus.write("Goal_Velocity", wname, cfg["moving_speed"], normalize=False)
        bus.write("Acceleration", wname, cfg["moving_acc"], normalize=False)
        bus.write("Torque_Limit", wname, ROTARY_TORQUE_LIMIT, normalize=False)
for wid in MOTOR_CONFIGS:
    bus.write("Torque_Enable", DISK_MOTOR_NAMES[wid], 1, normalize=False)


#------------------------------------------Threading Start-----------------------------------------------------#

posRotVariables = {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
state_lock = threading.Lock()
stop_event = threading.Event()

pygame.init()
pygame.display.set_mode((200, 200))

# pygame.joystick.init()

# while pygame.joystick.get_count() == 0:
#     print("NO CONTROLLER DETECTED. Please connect controller")

# PS4 = pygame.joystick.Joystick(0)
# PS4.init()
# print(f"Controller detected: {PS4.get_name()} ({PS4.get_numaxes()} axes)")
# pygame.event.pump()
# print(f"Axis values at startup (press L2/R2 to see which index moves): {[PS4.get_axis(i) for i in range(PS4.get_numaxes())]}")

# pygameThread = threading.Thread(target = getPS4Data, args = (state_lock, posRotVariables, stop_event, PS4))

pygameThread = threading.Thread(target = getPygameData, args = (state_lock, posRotVariables, stop_event))
pygameThread.start()

#-------------------------------------------MAIN FUNCTION------------------------------------------------------#

try:
    print("realTimeControl")
    p.setRealTimeSimulation(1)

    print (f"\nNumber of Joints: {p.getNumJoints(robot)}")
    getJointPositions(robot, movableJointIndices, True)
    tcpPosition, tcpRotQuat = getSimTCPPose(robot, TCPIndex, True)

    print("moving to start position")
    targetPosition = [tcpPosition[0], 0.0, 0.0]
    moveRobot(robot, TCPIndex, 
            movableJointIndices, 
            movableJointLowerLimits, 
            movableJointUpperLimits, 
            movableJointRanges,
            movableJointDampings,
            targetPosition,
            tcpRotQuat)

    time.sleep(5)
    getSimTCPPose(robot, TCPIndex, True)
    time.sleep(5)
    print("beginning teleop")
    while not stop_event.is_set():
        with state_lock:
            xDelta = posRotVariables["x"]
            yDelta = posRotVariables["y"]
            zDelta = posRotVariables["z"]
            rollDelta = posRotVariables["roll"]
            pitchDelta = posRotVariables["pitch"]
            yawDelta = posRotVariables["yaw"]

        tcpPosition, tcpRotQuat = getSimTCPPose(robot, TCPIndex)

        finalTargetPosition = [tcpPosition[0] + xDelta, tcpPosition[1] + yDelta, tcpPosition[2] + zDelta]

        rotQuatDelta = p.getQuaternionFromEuler([rollDelta, pitchDelta, yawDelta])
        _, finalTargetRotQuat = p.multiplyTransforms([0,0,0], tcpRotQuat, [0,0,0], rotQuatDelta)

        moveRobot(robot, TCPIndex, 
            movableJointIndices, 
            movableJointLowerLimits, 
            movableJointUpperLimits, 
            movableJointRanges,
            movableJointDampings,
            finalTargetPosition,
            finalTargetRotQuat)

            

except KeyboardInterrupt:
    print("KEYBOARD INTERRUPT DETECTED!")

finally:
    print("SHUTTING DOWN")
    stop_event.set()
    pygameThread.join()
    for wid in MOTOR_CONFIGS:
        bus.write("Torque_Enable", DISK_MOTOR_NAMES[wid], 0, normalize=False)
    p.disconnect()