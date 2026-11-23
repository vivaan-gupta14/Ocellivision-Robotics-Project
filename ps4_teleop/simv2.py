#------------------------------------------------------IMPORTS------------------------------------------------------#

import time
import numpy as np
import quaternion
import math
import threading

#PyBullet Imports
import pybullet as p
import pybullet_data

#Pygame Imports (for PS4 Controller)
import pygame

print("Imports completed")

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

def drawTCPFrame(robot, toolTipIndex, axisLength=0.05, lineWidth=3):
    axes = [
        ([axisLength, 0, 0], [1, 0, 0]),  # X - red
        ([0, axisLength, 0], [0, 1, 0]),  # Y - green
        ([0, 0, axisLength], [0, 0, 1]),  # Z - blue
    ]
    for endPoint, color in axes:
        p.addUserDebugLine(
            [0, 0, 0], endPoint, color,
            lineWidth=lineWidth,
            parentObjectUniqueId=robot,
            parentLinkIndex=toolTipIndex,
        )

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

#--------------------------------------------SET JOINT PROPERTIES-----------------------------------------------------
    
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

drawTCPFrame(robot, TCPIndex)

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
    p.disconnect()