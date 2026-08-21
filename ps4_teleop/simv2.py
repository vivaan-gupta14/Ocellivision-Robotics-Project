#------------------------------------------------------IMPORTS------------------------------------------------------#

import time
import numpy as np
import math

#PyBullet Imports
import pybullet as p
import pybullet_data

#Pygame Imports (for PS4 Controller)
import pygame

print("Imports completed")

#-------------------------------------------------THREADING FUNCTIONS------------------------------------------------------#

#Get's the PS4 controller data and updates variable dictionary using lock
def getPS4Data():
    return

def getPygameData()

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

def quatMultiply(q1, q2):
    # Hamilton product: rotate by q2 first, then q1 (both in the same reference frame).
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return [
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ]


def levelShaft(robot, toolTipIndex):
    # local Z of PSM1_roll_link is the shaft's pointing axis (roll spins about it). Rather
    # than guessing which Euler value corresponds to "horizontal" -- zeroing pitch turned
    # out to snap the shaft to fully vertical instead, since pitch=0 is this link's
    # straight-up rest orientation -- compute the minimal rotation that takes the shaft's
    # actual current world-space direction to its horizontal-plane projection directly.
    linkState = p.getLinkState(robot, toolTipIndex, computeForwardKinematics=1)
    position, currentQuat = linkState[4], linkState[5]

    rotMat = np.array(p.getMatrixFromQuaternion(currentQuat)).reshape(3, 3)
    shaftDir = rotMat @ np.array([0.0, 0.0, 1.0])

    horizontalDir = np.array([shaftDir[0], shaftDir[1], 0.0])
    horizontalDir /= np.linalg.norm(horizontalDir)

    dotVal = np.clip(np.dot(shaftDir, horizontalDir), -1.0, 1.0)
    angle = np.arccos(dotVal)
    axis = np.cross(shaftDir, horizontalDir)
    axisNorm = np.linalg.norm(axis)
    if axisNorm < 1e-9:
        deltaQuat = [0.0, 0.0, 0.0, 1.0]
    else:
        axis = axis / axisNorm
        s = np.sin(angle / 2)
        deltaQuat = [axis[0]*s, axis[1]*s, axis[2]*s, np.cos(angle/2)]

    targetQuat = quatMultiply(deltaQuat, currentQuat)
    return position, targetQuat

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

    POSITION_TOLERANCE_M = 0.01
    ROTATION_TOLERANCE_DEGREES = 2.0
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
# #-----------------------------------------------------------------TESTING EFFECT OF JOINT SETTINGS

position, orientation = p.getBasePositionAndOrientation(robot)
print(f"Base Position: {position}")
print(f"Base Orientation (Quaternion): {orientation}")

#-------------------------------------------MAIN FUNCTION------------------------------------------------------#

drawTCPFrame(robot, TCPIndex)

print("realTimeControl")
p.setRealTimeSimulation(1)

print("leveling shaft")
# levelPosition, levelRotQuat = levelShaft(robot, TCPIndex)
# moveRobot(robot, TCPIndex, 
#           movableJointIndices, 
#           movableJointLowerLimits, 
#           movableJointUpperLimits, 
#           movableJointRanges,
#           movableJointDampings,
#           levelPosition,
#           levelRotQuat)

time.sleep(10)

print (f"\nNumber of Joints: {p.getNumJoints(robot)}")
getJointPositions(robot, movableJointIndices, True)
tcpPosition, tcpRotQuat = getSimTCPPose(robot, TCPIndex, True)

print("moving")
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
getJointPositions(robot, movableJointIndices, True)
getSimTCPPose(robot, TCPIndex, True)
print("done")
time.sleep(20)
p.disconnect()