#-------------------------------------------CREATING THE SIMULATION-----------------------------------------------
from isaacsim.simulation_app import SimulationApp
sim = SimulationApp({"headless": False, "open_usd": "/home/rmf/MujocoAR/ur5_scene.usd", "display_options": 3286, "create_new_stage": False}) #Can replace usd with any other usd file
print("sim created with ur5_scene.usd loaded")

#-------------------------------------------------IMPORTS---------------------------------------------------------

import time
import threading
import numpy as np
from scipy.spatial.transform import Rotation as R
from mujoco_ar import MujocoARConnector
import rtde_control
import rtde_receive
import robotiq_gripper
import time
import pygame
from pygame.locals import *

#Isaac Sim Imports
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils import prims as prims_utils


#ROS Imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

print("Imports completed")

#-------------------------------------------THREADING FUNCTIONS-----------------------------------------------------

#Get phone data and update variable dictionary using lock
def getPhoneData(stop_event, lock, variables, phone):
    while not stop_event.is_set():
        phone_pose = phone.get_latest_data()
        if phone_pose is not None:
            with lock:
                variables['new_phone_position'] = np.asarray(phone_pose['position'])
                variables['new_phone_rotation_matrix'] = np.asarray(phone_pose['rotation'])
                variables['phone_toggle'] = phone_pose['toggle']
                variables['phone_button'] = phone_pose['button']
        time.sleep(0.002)

#Controls interaction with scale window (scroll to change scale, press R to reset scale, close window to stop progam
def changeScale(stop_event, lock, variables):
    pygame.init()
    screen = pygame.display.set_mode((500,500)) # Can change to any window size
    pygame.display.set_caption("Scroll in this window to adjust sensitivity")

    #Scaling variables
    min_scale = 0.1
    max_scale = 5
    scale_step = 0.1

    with lock:
        updateDisplay(screen, variables['scale'])

    while not stop_event.is_set():
        for event in pygame.event.get():
            #If window is closed, raise flag to shutdown program
            if event.type == pygame.QUIT:
                stop_event.set()
                break

            current_scale = 0 # Temporary local variable
            with lock:
                current_scale = variables['scale']

            #If mousewheel is scrolled, change scale up/down while staying in min/max limits
            if event.type == pygame.MOUSEWHEEL:
                if event.y > 0:
                    current_scale = min(current_scale + scale_step, max_scale)
                elif event.y < 0:
                    current_scale = max(current_scale - scale_step, min_scale)
                updateDisplay(screen, current_scale)

            if event.type == pygame.KEYDOWN and event.key == pygame.K_r:
                current_scale = 1.5
                updateDisplay(screen, current_scale)

            with lock:
                variables['scale'] = current_scale
        
        if stop_event.is_set():
            break

        time.sleep(0.01)

    pygame.quit()
           
#----------------------------------------------HELPER FUNCTIONS-----------------------------------------------------

#Updates scaling screen to display current scale
def updateDisplay(screen, scale):
    screen.fill((30,30,30))
    font = pygame.font.SysFont("Arial", 24)
    text_surface = font.render(f"Scale: {scale:.1f}  (Press 'R' to reset)", True, (255,255,255))
    screen.blit(text_surface, (10,35))
    pygame.display.flip()

#---------------------------------------------CREATING THE WORLD--------------------------------------------------

#Creates physics world in isaacsim
world = World(
    physics_dt= 1.0/500.0,  #500Hz (required for servoJ motion of robot in ursim)
    rendering_dt= 1.0/60.0, #60 FPS
    stage_units_in_meters=1.0, 
    set_defaults=True  
)
print("World object created")
print("Physics dt: ", world.get_physics_dt())
print("Rendering dt: ", world.get_rendering_dt())
world.reset()
world.step(render=True)
sim.update()

#Extended Kalman Filter to filter out noise and drift from phone position
def ekf_filter(position_measurement, dt, ekf_state, ekf_cov):
    """
    Filters 3D position using EKF with constant velocity model.

    Args:
        position_measurement (np.array): shape (3,), new position [x, y, z]
        dt (float): timestep in seconds (e.g., 1/500)
        ekf_state (np.array): shape (6, 1), current state [x, y, z, vx, vy, vz]
        ekf_cov (np.array): shape (6, 6), current covariance matrix

    Returns:
        filtered_position (np.array): shape (3,)
        new_ekf_state (np.array): shape (6, 1)
        new_ekf_cov (np.array): shape (6, 6)
    """
    # Predict
    F = np.eye(6)
    F[0, 3] = dt
    F[1, 4] = dt
    F[2, 5] = dt

    Q = np.eye(6) * 1e-4  # process noise
    pred_state = F @ ekf_state
    pred_cov = F @ ekf_cov @ F.T + Q

    # Measurement update
    H = np.zeros((3, 6))
    H[0, 0] = H[1, 1] = H[2, 2] = 1  # measuring only position

    R = np.eye(3) * 1e-2  # measurement noise
    z = position_measurement.reshape((3, 1))

    y = z - H @ pred_state
    S = H @ pred_cov @ H.T + R
    K = pred_cov @ H.T @ np.linalg.inv(S)

    updated_state = pred_state + K @ y
    updated_cov = (np.eye(6) - K @ H) @ pred_cov

    return updated_state[:3].flatten(), updated_state, updated_cov

#Extended Kalman Filter to filter out noise and drift from phone rotation
def ekf_filter_angular_motion(angular_velocity_measurement, dt, ekf_state_ang, ekf_cov_ang):
    """
    Filters 3D angular velocity using a 6-state EKF with a constant acceleration model.
    State: [wx, wy, wz, ax, ay, az] (angular velocity and angular acceleration)
    """
    # 1. PREDICTION STEP
    # State transition matrix F. Now it uses dt.
    # next_velocity = current_velocity + current_acceleration * dt
    F = np.eye(6)
    F[0:3, 3:6] = np.eye(3) * dt

    # Process noise Q (how much we expect the state to change randomly)
    # We add a tiny bit of noise to acceleration and a bit more to velocity.
    Q = np.diag([1e-4]*3 + [1e-3]*3) # Tuning knob for velocity and acceleration
    
    predicted_state = F @ ekf_state_ang
    predicted_cov = F @ ekf_cov_ang @ F.T + Q

    # 2. MEASUREMENT UPDATE STEP
    # Measurement matrix H (we are only measuring angular velocity)
    H = np.zeros((3, 6))
    H[0:3, 0:3] = np.eye(3)

    # Measurement noise R (how much we trust our raw angular velocity measurement)
    R_noise = np.eye(3) * 5e-2 # Tuning knob for measurement trust

    z = angular_velocity_measurement.reshape((3, 1))
    y = z - H @ predicted_state
    S = H @ predicted_cov @ H.T + R_noise
    K = predicted_cov @ H.T @ np.linalg.inv(S)

    updated_state = predicted_state + K @ y
    updated_cov = (np.eye(6) - K @ H) @ predicted_cov
    
    # Return the filtered angular velocity (the first 3 components of the state)
    return updated_state[:3].flatten(), updated_state, updated_cov


#-------------------------------CONNECTING TO URSIM ROBOT, URSIM GRIPPER, AND ISAACSIM ROBOT-------------------------------------

#Use this if using docker URSim
# rtde_c = rtde_control.RTDEControlInterface("192.168.56.101")
# rtde_r = rtde_receive.RTDEReceiveInterface("192.168.56.101")

#Use this if using actual UR Robot (Robot IP Should be 192.168.100.100)
rtde_c = rtde_control.RTDEControlInterface("192.168.100.150")
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.100.150")

print("Connected to URSim Robot")

#Creates a gripper controller
#Disable this if using docker
gripper = robotiq_gripper.RobotiqGripper()
gripper.connect("192.168.100.150", 63352)
gripper.activate()

print("Connected to Gripper")

#Creates Isaacsim robot controller
robot = Robot(prim_path="/ur5", name="UR5")
robot.initialize()

print("Connected to IsaacSim Robot")

#---------------------------------------------SETTING JOINT PROPERTIES------------------------------------------------------------

# The short names of the joints in isaacsim USD.
# The order MUST be consistent.
# Used for publishing joint information via ROS Node
# The joints that control the gripper are the finger_joint and the right_outer_knuckle_joint
ALL_JOINT_NAMES = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
    'finger_joint', 'right_outer_knuckle_joint'
]

# Get the indices for the gripper joints from the Isaac Sim articulation object.
# Used for publishing joint information via ROS Node
finger_joint_index = robot.get_dof_index(dof_name='finger_joint')
print(f"Found Isaac Sim gripper finger_joint index: {finger_joint_index}")
right_outer_knuckle_joint_index = robot.get_dof_index(dof_name='right_outer_knuckle_joint')
print(f"Found Isaac Sim gripper finger_joint index: {right_outer_knuckle_joint_index}")

#Identifies joint names and joint properties 
# print(robot.dof_names)
# print(robot.dof_properties.dtype.names)

#Set arm joints' stiffness and damping
arm_joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint"]
for arm_joint in arm_joint_names:
    arm_joint_path = f"/ur5/joints/{arm_joint}"
    prims_utils.set_prim_attribute_value(prim_path = arm_joint_path, attribute_name = "drive:angular:physics:stiffness", value = 6000)
    prims_utils.set_prim_attribute_value(prim_path = arm_joint_path, attribute_name = "drive:angular:physics:damping", value = 150.0)
    print("Stiffness after: ", prims_utils.get_prim_attribute_value(prim_path = arm_joint_path, attribute_name = "drive:angular:physics:stiffness"))
    print("Damping after: ", prims_utils.get_prim_attribute_value(prim_path = arm_joint_path, attribute_name = "drive:angular:physics:damping"))

#Sets wrist joints' stiffness and damping
wrist_joint_names = ["wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
for wrist_joint in wrist_joint_names:
    wrist_joint_path = f"/ur5/joints/{wrist_joint}"
    prims_utils.set_prim_attribute_value(prim_path = wrist_joint_path, attribute_name = "drive:angular:physics:stiffness", value = 4000)
    prims_utils.set_prim_attribute_value(prim_path = wrist_joint_path, attribute_name = "drive:angular:physics:damping", value = 100.0)
    print("Stiffness after: ", prims_utils.get_prim_attribute_value(prim_path = wrist_joint_path, attribute_name = "drive:angular:physics:stiffness"))
    print("Damping after: ", prims_utils.get_prim_attribute_value(prim_path = wrist_joint_path, attribute_name = "drive:angular:physics:damping"))

#Identifies joint properties for gripper joints
# print(prims_utils.get_prim_attribute_names("/ur5/gripper_tutorial_version/Robotiq_2F_85/finger_joint"))

#Sets finger joint properties
prims_utils.set_prim_attribute_value(prim_path = "/ur5/gripper_tutorial_version/Robotiq_2F_85/finger_joint", attribute_name = "drive:angular:physics:stiffness", value = 0.0)
prims_utils.set_prim_attribute_value(prim_path = "/ur5/gripper_tutorial_version/Robotiq_2F_85/finger_joint", attribute_name = "drive:angular:physics:damping", value = 5000.0)
prims_utils.set_prim_attribute_value(prim_path = "/ur5/gripper_tutorial_version/Robotiq_2F_85/finger_joint", attribute_name = "drive:angular:physics:maxForce", value = 30.0)
prims_utils.set_prim_attribute_value(prim_path = "/ur5/gripper_tutorial_version/Robotiq_2F_85/finger_joint", attribute_name = "physxJoint:maxJointVelocity", value = 130.0)
prims_utils.set_prim_attribute_value(prim_path = "/ur5/gripper_tutorial_version/Robotiq_2F_85/finger_joint", attribute_name = "physics:lowerLimit", value = 0.0)
prims_utils.set_prim_attribute_value(prim_path = "/ur5/gripper_tutorial_version/Robotiq_2F_85/finger_joint", attribute_name = "physics:upperLimit", value = 75.0)

#Sets right outer knuckle joint properties
prims_utils.set_prim_attribute_value(prim_path = "/ur5/gripper_tutorial_version/Robotiq_2F_85/right_outer_knuckle_joint", attribute_name = "drive:angular:physics:stiffness", value = 0.0)
prims_utils.set_prim_attribute_value(prim_path = "/ur5/gripper_tutorial_version/Robotiq_2F_85/right_outer_knuckle_joint", attribute_name = "drive:angular:physics:damping", value = 5000.0)
prims_utils.set_prim_attribute_value(prim_path = "/ur5/gripper_tutorial_version/Robotiq_2F_85/right_outer_knuckle_joint", attribute_name = "drive:angular:physics:maxForce", value = 30.0)
prims_utils.set_prim_attribute_value(prim_path = "/ur5/gripper_tutorial_version/Robotiq_2F_85/right_outer_knuckle_joint", attribute_name = "physxJoint:maxJointVelocity", value = 130.0)
prims_utils.set_prim_attribute_value(prim_path = "/ur5/gripper_tutorial_version/Robotiq_2F_85/right_outer_knuckle_joint", attribute_name = "physics:lowerLimit", value = 0.0)
prims_utils.set_prim_attribute_value(prim_path = "/ur5/gripper_tutorial_version/Robotiq_2F_85/right_outer_knuckle_joint", attribute_name = "physics:upperLimit", value = 75.0)

#Sets left outer finger joint properties
prims_utils.set_prim_attribute_value(prim_path = "/ur5/gripper_tutorial_version/Robotiq_2F_85/left_outer_finger_joint", attribute_name = "drive:angular:physics:stiffness", value = 0.05)
prims_utils.set_prim_attribute_value(prim_path = "/ur5/gripper_tutorial_version/Robotiq_2F_85/left_outer_finger_joint", attribute_name = "physics:lowerLimit", value = 0.0)
prims_utils.set_prim_attribute_value(prim_path = "/ur5/gripper_tutorial_version/Robotiq_2F_85/left_outer_finger_joint", attribute_name = "physics:upperLimit", value = 180.0)

#Sets right outer finger joint properties
prims_utils.set_prim_attribute_value(prim_path = "/ur5/gripper_tutorial_version/Robotiq_2F_85/right_outer_finger_joint", attribute_name = "drive:angular:physics:stiffness", value = 0.05)
prims_utils.set_prim_attribute_value(prim_path = "/ur5/gripper_tutorial_version/Robotiq_2F_85/right_outer_finger_joint", attribute_name = "physics:lowerLimit", value = 0.0)
prims_utils.set_prim_attribute_value(prim_path = "/ur5/gripper_tutorial_version/Robotiq_2F_85/right_outer_finger_joint", attribute_name = "physics:upperLimit", value = 180.0)


remaining_gripper_joint_paths = [
    "/ur5/gripper_tutorial_version/Robotiq_2F_85/right_inner_finger_joint",
    "/ur5/gripper_tutorial_version/Robotiq_2F_85/right_inner_knuckle_joint",
    "/ur5/gripper_tutorial_version/Robotiq_2F_85/right_inner_finger_knuckle_joint",
    "/ur5/gripper_tutorial_version/Robotiq_2F_85/left_inner_finger_knuckle_joint",
    "/ur5/gripper_tutorial_version/Robotiq_2F_85/left_inner_finger_joint",
    "/ur5/gripper_tutorial_version/Robotiq_2F_85/left_inner_knuckle_joint"
]

#Sets the limit for all other joints to (basically) unlimited
for gripper_joints in remaining_gripper_joint_paths:
    prims_utils.set_prim_attribute_value(prim_path = gripper_joints, attribute_name = "physics:lowerLimit", value = 1.0)
    prims_utils.set_prim_attribute_value(prim_path = gripper_joints, attribute_name = "physics:upperLimit", value = -1.0)

print("Robot properties initialized")


#---------------------------------------------INITIALIZING VARIABLES---------------------------------------------------------------

#Obtains the starting position and rotation of robot TCPs
home_tcp_pose = np.asarray(rtde_r.getActualTCPPose())
prev_tcp_position = home_tcp_pose[:3]
prev_tcp_rotation_rotvec = home_tcp_pose[3:]
print("Initial TCP Position: ", prev_tcp_position)
print("Initial TCP Rotation: ", prev_tcp_rotation_rotvec)

#Teleporting Isaac Simrobot to start position by reading joint values from URSim
home_joint_positions = rtde_r.getActualQ()
print("Applying URSim joint positions to Isaac Sim robot...")
robot.set_joint_positions(home_joint_positions, joint_indices=np.arange(6)) #First 6 joints are the arm, rest are the gripper. Only want to move the arm joints

# Step the physics forward a few times to ensure the simulation state is consistent
for _ in range(10):
    world.step(render=True)
    sim.update()

print("Isaac Sim and URSim Robots Synced")
world.step(render=True)
time.sleep(0.1)

# Dictionary that stores variables that are used by threads
variables = {"new_phone_position": None, "new_phone_rotation_matrix": None, "phone_toggle": False, "phone_button": False, "scale": 1.5}

# Because threads operate simultaneously, need to make sure only one thread can access variables at a time using the lock
state_lock = threading.Lock()

#Flag to shut down threads and stop program
stop_event = threading.Event()

#Extended Kalman Filter variables
ekf_state = np.zeros((6, 1))         # Initial position and velocity
ekf_cov = np.eye(6) * 1e-3           # Initial uncertainty
ekf_state_angular = np.zeros((6,1))
ekf_cov_angular = np.eye(6) * 1e-3
last_ekf_time = time.time()

#The mode that the robot is operating in (TELEOP or HOMING)
control_mode = "TELEOP"

#--------------------------------------------ROS 2 INITIALIZATION--------------------------------------------------

#Creates a ROS Node to publish joint information
print("Initializing ROS 2...")
rclpy.init()
ros_node = Node("ur5_teleop_publisher")
# This topic name is what the recorder extension will listen to.
joint_state_publisher = ros_node.create_publisher(JointState, "/follower/joint_states", 10)
print("ROS 2 Node and Publisher for '/follower/joint_states' created.")


#--------------------------------------------CONNECTING TO PHONE-----------------------------------------------------

#Connects to phone MujocoAR app at port 8888 (ensure that this port is not blocked by firewall)
phone = MujocoARConnector(port=8888, debug=False)
phone.start()
print("Waiting for device to connect...")
while not phone.connected_clients:
    time.sleep(0.002)
print("MujocoAR Initialized")
#Waits for some time to wait for phone to start sending data
time.sleep(5)

#------------------------------------------------THREADING-----------------------------------------------------------

#Start the phone and scaling threads
phone_thread = threading.Thread(target = getPhoneData, args=(stop_event, state_lock, variables, phone))
scale_thread = threading.Thread(target = changeScale, args=(stop_event, state_lock, variables))
phone_thread.start()
scale_thread.start()

#------------------------------------INITIALIZING FIRST PHONE VARIABLES----------------------------------------------

while True:
    with state_lock:
        if variables['new_phone_position'] is not None:
            prev_phone_position = variables['new_phone_position']
            prev_phone_rotation_matrix = variables['new_phone_rotation_matrix']
            break
    time.sleep(0.1)

#--------------------------------------------CONTROL LOOP------------------------------------------------------------

try:
    while sim.is_running() and not stop_event.is_set():
        #1. Read the new phone data
        with state_lock:
            current_phone_position = variables["new_phone_position"].copy()
            current_phone_rotation_matrix = variables['new_phone_rotation_matrix'].copy()
            current_scale = variables['scale']
            current_toggle = variables['phone_toggle']
            current_button = variables['phone_button']

        #Switches to homing if button is pressed
        if current_button and control_mode == "TELEOP":
            print("Switching to HOMING mode...")
            control_mode = "HOMING"

        #Moves robot to original starting position
        if control_mode == "HOMING":
            rtde_c.servoJ(home_joint_positions, 0, 0, 0.002, 0.1, 300)
            current_joint_positions = rtde_r.getActualQ()
            robot.apply_action(ArticulationAction(joint_positions=current_joint_positions, joint_indices = np.array([0,1,2,3,4,5])))
            
            # Check if we have arrived at the home position
            actual_pose = np.asarray(rtde_r.getActualTCPPose())
            distance_to_home = np.linalg.norm(home_tcp_pose[:3] - actual_pose[:3])
            
            # If we are very close to home (e.g., within 1mm)
            if distance_to_home < 0.001: 
                print("Homing complete. Switching back to TELEOPERATION mode.")
                control_mode = "TELEOP"
                # CRITICAL: Reset the "previous" poses to prevent a jump when teleop resumes!
                prev_phone_position = current_phone_position
                prev_phone_rotation_matrix = current_phone_rotation_matrix
                prev_tcp_position = actual_pose[:3]
                prev_tcp_rotation_rotvec = actual_pose[3:]
        
        elif control_mode == "TELEOP":
            #2. Calculate position and rotation deltas
            pos_delta_raw = current_phone_position - prev_phone_position

            r_new_phone = R.from_matrix(current_phone_rotation_matrix)
            r_old_phone = R.from_matrix(prev_phone_rotation_matrix)
            r_rot_delta = r_new_phone * r_old_phone.inv()

            #3. Filter position delta
            current_time = time.time()
            dt = max(current_time - last_ekf_time, 1e-6)
            last_ekf_time = current_time
            pos_delta_filtered, ekf_state, ekf_cov = ekf_filter(pos_delta_raw, dt, ekf_state, ekf_cov)

            angular_velocity_raw = r_rot_delta.as_rotvec() / dt
            angular_velocity_filtered, ekf_state_angular, ekf_cov_angular = ekf_filter_angular_motion(angular_velocity_raw, dt, ekf_state_angular, ekf_cov_angular)
            rot_delta_filtered_rotvec = angular_velocity_filtered * dt

            #4. Scale position and rotation deltas
            pos_delta_scaled = pos_delta_filtered * current_scale

            rot_delta_scaled_rotvec = rot_delta_filtered_rotvec * current_scale

            #5. Swap and invert positions and rotations
            pos_delta_swapped = np.array([-pos_delta_scaled[1], pos_delta_scaled[0], pos_delta_scaled[2]])
            
            rot_delta_swapped_rotvec = np.array([rot_delta_scaled_rotvec[1], -rot_delta_scaled_rotvec[0], rot_delta_scaled_rotvec[2]])

            #6. Add to old tcp position and rotation
            final_target_pos = prev_tcp_position + pos_delta_swapped

            r_rot_delta_swapped = R.from_rotvec(rot_delta_swapped_rotvec)
            r_old_tcp = R.from_rotvec(prev_tcp_rotation_rotvec)
            r_final_target_rot = r_rot_delta_swapped * r_old_tcp
            final_target_rot_rotvec = r_final_target_rot.as_rotvec()

            #7. Get IK Solution and Command Both Robots
            target_tcp_pose = np.concatenate((final_target_pos, final_target_rot_rotvec))
            target_pose_joints = rtde_c.getInverseKinematics(target_tcp_pose)
            if target_pose_joints:
                rtde_c.servoJ(target_pose_joints, 0, 0, 0.002, 0.1, 300)
                current_ursim_joints = rtde_r.getActualQ()
                action = ArticulationAction(joint_positions = current_ursim_joints, joint_indices = np.array([0,1,2,3,4,5]))
                robot.apply_action(action)

                # 1. Get arm positions from the "master" simulation (URSim)
                arm_joint_positions = current_ursim_joints # This is the array of 6 from rtde_r

                # 2. Get gripper positions from the "slave" simulation (Isaac Sim)
                # Get ALL joint positions from the Isaac Sim robot
                all_isaac_joint_positions = robot.get_joint_positions()
                # Select only the gripper positions using the indices we found earlier
                finger_joint_position = all_isaac_joint_positions[finger_joint_index]
                right_outer_knuckle_joint_position = all_isaac_joint_positions[right_outer_knuckle_joint_index]

                # 3. Combine into a single array in the correct order
                all_8_joint_positions = np.concatenate((arm_joint_positions, [finger_joint_position], [right_outer_knuckle_joint_position]))

                # 4. Construct and publish the ROS 2 message
                joint_state_msg = JointState()
                joint_state_msg.header = Header()
                joint_state_msg.header.stamp = ros_node.get_clock().now().to_msg()
                
                # Use the full list of 8 joint names
                joint_state_msg.name = ALL_JOINT_NAMES
                # Use the combined list of 8 joint positions
                joint_state_msg.position = all_8_joint_positions.tolist()
                
                joint_state_publisher.publish(joint_state_msg)


            else:
                print("Cannot compute IK")

            #8. Update variables
            prev_phone_position = current_phone_position
            prev_phone_rotation_matrix = current_phone_rotation_matrix

            actual_pose = np.asarray(rtde_r.getActualTCPPose())
            prev_tcp_position = actual_pose[:3]
            prev_tcp_rotation_rotvec = actual_pose[3:]

            #9. Gripper Control via Phone Toggle
            if current_toggle:
                #Disable this if using docker
                gripper.move_and_wait_for_pos(255, 255, 255)

                prims_utils.set_prim_attribute_value(prim_path = "/ur5/gripper_tutorial_version/Robotiq_2F_85/finger_joint", attribute_name = "drive:angular:physics:targetVelocity", value = 80.0)
                prims_utils.set_prim_attribute_value(prim_path = "/ur5/gripper_tutorial_version/Robotiq_2F_85/right_outer_knuckle_joint", attribute_name = "drive:angular:physics:targetVelocity", value = 80.0)

            else:
                #disable this if using docker
                gripper.move_and_wait_for_pos(0, 255, 255)

                prims_utils.set_prim_attribute_value(prim_path = "/ur5/gripper_tutorial_version/Robotiq_2F_85/finger_joint", attribute_name = "drive:angular:physics:targetVelocity", value = -80.0)
                prims_utils.set_prim_attribute_value(prim_path = "/ur5/gripper_tutorial_version/Robotiq_2F_85/right_outer_knuckle_joint", attribute_name = "drive:angular:physics:targetVelocity", value = -80.0)


        #10. Update Isaac Sim
        world.step(render=True)
        sim.update()
        rclpy.spin_once(ros_node, timeout_sec=0)
        time.sleep(0.002)

except KeyboardInterrupt:
    print("KEYBOARD INTERRUPT DETECTED")

finally:
    print("Shutting down...")
    print("Shutting down ROS 2...")
    ros_node.destroy_node()
    rclpy.shutdown()
    print("Shutting down...")
    print("Threads ending their loops...")
    stop_event.set()
    print("Robot stopping movement...")
    rtde_c.servoStop()
    print("Robot controller disconnecting...")
    rtde_c.disconnect()
    print("Robot receiver disconnecting...")
    rtde_r.disconnect()
    print("Phone thread rejoining...")
    phone_thread.join()
    print("Scaling thread rejoining...")
    scale_thread.join()
    print("All threads stopped successfully")
    print("Program Stopped")