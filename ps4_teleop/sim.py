#-------------------------------------------------IMPORTS---------------------------------------------------------

import pybullet as p
import time
import pybullet_data
import numpy as np

#---------------------------------------------CREATING THE WORLD--------------------------------------------------

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, 0)

planeID = p.loadURDF("plane.urdf")
robot = p.loadURDF("/home/user/Desktop/Vivaan Internship Project/Ocellivision-Robotics-Project/Robot Files/full_assembly.urdf", baseOrientation=p.getQuaternionFromEuler([0, 0, 0]), useFixedBase=1)
p.setRealTimeSimulation(0)

#---------------------------------------------HELPER FUNCTIONS-------------------------------------------------

def getTCPPose(robot, toolTipIndex):
    linkState = p.getLinkState(robot, toolTipIndex, computeForwardKinematics=1)
    tcpPosition = linkState[4]
    tcpQuaternion = linkState[5]
    print(f"TCP position: {tcpPosition}")
    print(f"TCP orientation (quaternion): {tcpQuaternion}")
    print(f"TCP orientation (Euler angles): {p.getEulerFromQuaternion(tcpQuaternion)}")
    return tcpPosition, tcpQuaternion, p.getEulerFromQuaternion(tcpQuaternion)

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
    return position, p.getEulerFromQuaternion(targetQuat)

def moveRobot(robot, targetPosition, targetEulerOrientation = None):
    print("reading current joint positions...")
    currentJointPositions = [p.getJointState(robot, i)[0] for i in allMovableJointIndices]

    print("creating target joint positions...")

    if targetEulerOrientation is None:
        _, quaternion, _ = getTCPPose(robot, toolTipIndex)
        targetJointPositions = p.calculateInverseKinematics(
                robot, toolTipIndex, targetPosition,targetOrientation = quaternion,
                lowerLimits = minJointLimits,
                upperLimits = maxJointLimits,
                jointRanges = jointRanges,
                restPoses = currentJointPositions,
                jointDamping = jointDamping,
                solver = p.IK_SDLS,
                maxNumIterations = 500,
                residualThreshold = 1e-8,)
    else:
        levelQuat = p.getQuaternionFromEuler(levelEuler)
        deltaQuat = p.getQuaternionFromEuler(targetEulerOrientation)
        targetQuat = quatMultiply(deltaQuat, levelQuat)
        targetJointPositions = p.calculateInverseKinematics(
            robot, toolTipIndex, targetPosition,targetOrientation = targetQuat,
            lowerLimits = minJointLimits,
            upperLimits = maxJointLimits,
            jointRanges = jointRanges,
            restPoses = currentJointPositions,
            jointDamping = jointDamping,
            solver = p.IK_SDLS,
            maxNumIterations = 500,
            residualThreshold = 1e-8,)

    print("moving robot")
    p.setJointMotorControlArray(
        robot, allMovableJointIndices, p.POSITION_CONTROL,
        targetPositions=targetJointPositions,
        positionGains=[0.02]*len(allMovableJointIndices),
        velocityGains=[1.0]*len(allMovableJointIndices),
        forces=[500]*len(allMovableJointIndices))


#---------------------------------------------JOINT SETUP-------------------------------------------------

jointNameToIndex = {} #maps joint names to their indices
manualJointNames = {"jaw_1", "jaw_2", "carriage_prismatic_joint", "roll", "wrist_pitch"}
allMovableJointIndices = [] #every non-fixed joint, ascending order -- matches calculateInverseKinematics's return order
IKJointIndices = [] #fed to ik solver
manualJointIndices = [] #directly controlled with PS4

for i in range(p.getNumJoints(robot)):
    jointInfo = p.getJointInfo(robot, i)
    jointName = jointInfo[1].decode('utf-8')
    jointType = jointInfo[2]
    childLinkName = jointInfo[12].decode('utf-8')
    # toolTipIndex targets PSM1_roll_link (the "roll" joint's child) instead of the actual
    # tool tip -- that's the point where the shaft meets the wrist/jaws, with zero offset
    # from where wrist_pitch is measured. IK physically cannot move wrist_pitch (or
    # wrist_yaw/jaw_1/jaw_2, all downstream of it) to affect this link's pose, since they
    # sit past it in the chain -- no damping/exclusion trick needed for that joint anymore.
    if childLinkName == "PSM1_roll_link":
        toolTipIndex = i
    if jointType != p.JOINT_FIXED:
        jointNameToIndex[jointName] = i
        allMovableJointIndices.append(i)
        if jointName in manualJointNames:
            manualJointIndices.append(i)
        else:
            IKJointIndices.append(i)

# calculateInverseKinematics solves over every non-fixed joint in the body (not just
# IKJointIndices), so limits/damping/restPoses must all be sized to allMovableJointIndices.
# roll and carriage_prismatic_joint sit on the kinematic chain to the tool tip, so IK can
# still move them -- give them high damping so it only does so as a last resort, leaving
# them to manual PS4 control the rest of the time. jaw_1/jaw_2 are off-chain (branch to the
# gripper, not an ancestor of the tool tip) so IK can't move them regardless of damping.
IK_JOINT_DAMPING = 0.1
MANUAL_JOINT_DAMPING = 2000.0

minJointLimits = []
maxJointLimits = []
jointRanges = []
jointDamping = []
for i in allMovableJointIndices:
    jointInfo = p.getJointInfo(robot, i)
    minJointLimits.append(jointInfo[8])
    maxJointLimits.append(jointInfo[9])
    jointRanges.append(jointInfo[9] - jointInfo[8])
    if i in manualJointIndices:
        jointDamping.append(MANUAL_JOINT_DAMPING)
    else:
        jointDamping.append(IK_JOINT_DAMPING)

minManualJointLimits = []
maxManualJointLimits = []
for i in manualJointIndices:
    jointInfo = p.getJointInfo(robot, i)
    minManualJointLimits.append(jointInfo[8])
    maxManualJointLimits.append(jointInfo[9])


startingJointPositions = {
    "shoulder_pan": 0.0,
    "shoulder_lift": -1.74533,
    "elbow_flex": 1.57,
    "wrist_flex": -1.65806,
    "wrist_to_rail": 0.0,
    "carriage_prismatic_joint": 0.0,
    "roll": 0.0,
    "wrist_pitch": 0.0,
    "jaw_1": 0.0,
    "jaw_2": 0.0
}


for name, position in startingJointPositions.items():
    p.resetJointState(robot, jointNameToIndex[name], position)

drawTCPFrame(robot, toolTipIndex)

#---------------------------------------------LEVELING THE SHAFT----------------------------------------------------
p.setRealTimeSimulation(1)
time.sleep(10)

levelPosition, levelEuler = levelShaft(robot, toolTipIndex)

moveRobot(robot, [levelPosition[0], 0.0, levelPosition[2]], [0.0, 0.0, 0.0])
time.sleep(5)
getTCPPose(robot, toolTipIndex)

#---------------------------------------------CONTROL LOOP----------------------------------------------------

print("done moving robot, sleeping for 60 seconds...")
time.sleep(60)