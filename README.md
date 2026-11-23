# Ocellivision Robotics Project — Handoff Documentation

*Author: Vivaan Gupta (A\*STAR internship). Last updated 23 November 2026. Companion to the repository [Ocellivision-Robotics-Project](https://github.com/vivaan-gupta14/Ocellivision-Robotics-Project).*

## Project Overview

A hybrid surgical-style robot that combines an SO-101 robotic arm, a belt-driven prismatic carriage, and a da Vinci EndoWrist instrument (8 mm graspers). The goal is to pass the instrument through a small opening without touching its sides (Remote Center of Motion, RCM) and pick and place tissue samples, as a step toward automating tissue preparation for histopathology.

All 10 FeeTech STS3215 servos are daisy-chained on one serial bus:

- **Motors 1–5:** SO-101 arm (shoulder\_pan, shoulder\_lift, elbow\_flex, wrist\_flex, wrist\_roll)
- **Motor 6:** prismatic carriage (GT2 belt drive, multi-turn mode, sensorless homing)
- **Motors 7–10:** da Vinci instrument disks (wrist\_pitch, roll, jaw\_1, jaw\_2), calibrated so center = 2048

## Current Status

- **Working on hardware:** leader–follower teleop of the arm plus keyboard jogging of motors 6–10 (`07_teleoperate_full_10dof.py`).
- **Working in simulation:** PyBullet IK teleop of the full robot model from the keyboard (`simv2.py`).
- **Stage 1 only:** the PS4 controller is read and its inputs displayed (`controller_input.py`), but it is not yet connected to the simulation or the robot.
- **Not working:** sim-to-real transfer (`simHardware.py`). RCM control is not implemented.
- **Hardware:** the prismatic carriage, motor adapters and instrument mount are designed, printed and assembled (see Mechanical design).

## Reading Order

Read **Known Issues** before running anything on the hardware. Then read these, in this order. Everything else in the repo is an earlier version, an experiment or vendor example code (see Archived experiments).

1. `Robot Files/full_assembly.urdf`: the robot model used by every simulation script.
2. `so101_lerobot/scripts/commands.md`: how to bring the robot up, step by step.
3. `so101_lerobot/so101_follower_5dof.py`: the 5-motor LeRobot follower class.
4. `so101_lerobot/scripts/07_teleoperate_full_10dof.py`: the reference for running the full robot on hardware.
5. `STServo_Python/stservo-env/sms_sts/calibration.py` and `positionCalibration.py` (with `motor.py`): register-level homing of the carriage and centering of the instrument motors.
6. `ps4_teleop/pastProject.py`, then `ps4_teleop/simv2.py`: the reference logic, then the working IK simulation built on it.
7. `ps4_teleop/simHardware.py`: the unfinished sim-to-real attempt.
8. Mechanical design (CAD): how the carriage and motor adapters are built and assembled.

Supporting references: `torque_test_data/` (torque test conclusions) and `ST3215 memory register map-EN.xls`.

## Repository Layout

| Folder | What it holds |
| --- | --- |
| `Robot Files/` | The merged URDF (`full_assembly.urdf`) and its meshes. |
| `so101_lerobot/` | LeRobot setup, calibration and teleop scripts for the SO-101 arms, plus script 07 for all 10 motors. |
| `STServo_Python/stservo-env/sms_sts/` | Register-level scripts for motors 6–10 (homing, centering, torque testing, utilities), next to FeeTech's SDK (`scservo_sdk/`). |
| `ps4_teleop/` | PS4 controller input, the PyBullet simulation, the sim-to-real attempt and the UR5 reference script. |
| CAD files | Fusion 360 designs and STL exports of the adapters, carriage and RCM stand. |

`so101_lerobot/`, `STServo_Python/` and `ps4_teleop/` each have their own Python environment. Set each one up as described in its section, and run scripts from inside their own folder.

## `Robot Files/`: Robot Model

### `full_assembly.urdf`

**Purpose:** `full_assembly.urdf` combines the SO-101 arm, the prismatic carriage and the da Vinci instrument into one robot model. PyBullet loads it for simulation and inverse kinematics (IK).

**Why simulation first:** IK and control logic were developed in simulation so they could be tested with no risk to the hardware before running on the physical robot. NVIDIA Isaac Sim would have been the better simulator, but the development laptop had no NVIDIA GPU. PyBullet was used instead because it gives a visual simulation and has a built-in IK solver.

**How it was built.** The file merges three source models:

1. **SO-101 arm:** the public SO-101 URDF (`so101_new_calib.urdf`, generated with onshape-to-robot), with the gripper and the last joints removed to match the hardware actually used.
2. **Prismatic carriage:** the source is the project's own Fusion 360 assembly. Joints were not defined in Fusion. The model was converted from Fusion to URDF with a Fusion 360 add-in (websites that do the conversion also exist; search "Fusion 360 to URDF"), and the joint values were derived and adjusted by hand.
3. **da Vinci instrument (PSM1):** an open-source model, believed to come from a Johns Hopkins research repository. The shaft and tip differ from the instruments in the lab, but the housing and the interface that mounts to the carriage match.

The merged model was checked in the web URDF viewer [viewer.robotsfan.com](https://viewer.robotsfan.com). Joint limits, origins and overlapping geometry were corrected by editing the URDF by hand until the model matched the real robot.

**Joint map (movable joints):**

| URDF joint | Type | Limits | Real motor |
| --- | --- | --- | --- |
| `shoulder_pan` | revolute | ±1.92 rad | 1 |
| `shoulder_lift` | revolute | −1.82 to 1.75 rad | 2 |
| `elbow_flex` | revolute | −1.69 to 1.69 rad | 3 |
| `wrist_flex` | revolute | −1.86 to 1.66 rad | 4 |
| `wrist_to_rail` | revolute | ±1.25 rad | 5 (called `wrist_roll` in LeRobot; hardware sign is inverted) |
| `carriage_prismatic_joint` | prismatic | −0.0625 to 0.1131 m | 6 |
| `wrist_pitch` | revolute | ±1.40 rad | 7 |
| `roll` | revolute | ±4.54 rad | 8 |
| `jaw_1` | revolute | ±1.57 rad | 9 |
| `jaw_2` | revolute | ±1.57 rad | 10 |

**Important links:** `PSM1_tool_tip_link` is the tool center point (TCP) used in `simv2.py`. `wrist_roll_pivot_link` and `PSM1_roll_link` are useful intermediate IK targets.

**Known limitations:**

- `wrist_yaw` is a fixed joint, so the model has no yaw degree of freedom at the instrument.
- The PSM1 links have no mass or inertia data. The roughly 40 g instrument shaft has no link that carries its mass.
- Mesh paths are relative (`assets/`), but every sim script loads the URDF from a hard-coded absolute path (`/home/user/Desktop/...`), which must be changed on a new machine.
- The header comment says wrist\_roll was replaced by a fixed mount, but `wrist_to_rail` is revolute and maps to motor 5. The comment is out of date.
- Joint limits for shoulder\_lift, elbow\_flex and wrist\_flex were checked against the real arm with `simHardwareSync.py`, which is not committed (see Known Issues).

## `so101_lerobot/`: LeRobot Control of the Arm

Scripts for setting up, calibrating and teleoperating the SO-101 arms, built on Hugging Face's LeRobot library, plus script 07 for the full 10-motor robot. Run everything from inside `so101_lerobot/`.

### Setup: the `so101_lerobot` environment

`so101_lerobot/` is a self-contained [uv](https://docs.astral.sh/uv/) project with its own Python environment, separate from `STServo_Python/` and `ps4_teleop/`. It needs Python 3.12 and `lerobot[feetech,core_scripts]`, pinned in `pyproject.toml` and `uv.lock`.

```
cd so101_lerobot
uv sync
```

This creates `so101_lerobot/.venv`, which the shell scripts call directly (`.venv/bin/lerobot-...`). Run every script from inside `so101_lerobot/` so relative paths resolve. On Linux, if a port gives a permission error, add your user to the `dialout` group and log in again.

The folder's own `README.md` explains why the follower uses a 5-motor class and lists the bring-up order. Read it alongside this section.

### `so101_follower_5dof.py`

**Purpose:** `SO101Follower5DOF` is a subclass of LeRobot's standard SO-101 follower that overrides its motor list so the follower arm has five motors instead of six. The leader arm is unchanged and still has six motors. The follower's gripper was removed, and its ID 6 is now the prismatic carriage.

**Why it is needed:** the standard `SOFollower` class hard-codes six motors, with a gripper on ID 6. Using it would send gripper commands to the carriage motor and conflict with the code that controls motors 6–10.

**What it changes:**

- It skips the parent's `__init__` and builds a `FeetechMotorsBus` with only IDs 1–5 (shoulder\_pan, shoulder\_lift, elbow\_flex, wrist\_flex, wrist\_roll).
- Everything else (calibrate, configure, setup\_motors, get\_observation, send\_action) is inherited unchanged, because those methods loop over whatever motors the bus has.
- It sets `name = "so101_follower_5dof"`, so its calibration is saved to `~/.cache/huggingface/lerobot/calibration/robots/so101_follower_5dof/<id>.json` and can never overwrite a standard 6-motor calibration file.

**Used by:** `04_setup_motors_follower.py`, `05_calibrate_follower.py`, `06_teleoperate_base_arm.py`, `07_teleoperate_full_10dof.py` (to load calibration only) and `ps4_teleop/simHardware.py`. A copy is kept in `ps4_teleop/` so that folder can run on its own.

**Notes:** the class is created directly in these scripts. It is not registered with LeRobot's command-line tools, so `--robot.type=so101_follower_5dof` will not work.

### Running the robot: `scripts/commands.md`

**Purpose:** `commands.md` is the cheat sheet for running the scripts in the right order. Follow it top to bottom each session.

**One-time setup (not in `commands.md`):** scripts `02_setup_motors_leader.sh` and `04_setup_motors_follower.py` assign each motor its ID (1–6 on the leader, 1–5 on the follower). The ID is stored permanently in the servo's EEPROM, so these run only once: when an arm is first built or a motor is replaced. Connect one motor at a time as prompted. Script 04 only assigns IDs 1–5, so it never touches motors 6–10. Motors 6–10 were given their IDs with `change_id.py` in `STServo_Python/stservo-env/sms_sts/`.

**Step 1: find the ports.** Run `scripts/01_find_ports.sh`. It walks you through unplugging each arm's USB cable so it can tell which port belongs to which arm. The leader port connects the laptop to the leader arm; the follower port connects it to the follower arm (and all 10 follower motors). On the original setup, the leader was `/dev/ttyACM1` and the follower was `/dev/ttyACM0`. These change between machines and sometimes between reboots, so treat every port in the commands as a placeholder.

**Step 2: calibrate the leader arm.**

```
scripts/03_calibrate_leader.sh <leader-port> leader_arm
```

`leader_arm` is the ID the calibration file is saved under. Use the same ID later in script 06 or 07. LeRobot's calibration prompts you to put the arm in the middle of its range and press Enter, then to move every joint through its full range.

**Step 3: calibrate the follower arm (motors 1–5).**

```
uv run scripts/05_calibrate_follower.py <follower-port> follower_arm
```

Same process as the leader. `follower_arm` must also match what you pass to 06 or 07.

Both calibration scripts are built on LeRobot's own calibration tools. `03_calibrate_leader.sh` is a thin wrapper that runs the standard `lerobot-calibrate` command for a 6-motor SO-101 leader. `05_calibrate_follower.py` is a modified version of LeRobot's follower calibration: it uses `SO101Follower5DOF`, so it calibrates only motors 1–5 and never touches motors 6–10.

**Step 4: set up motors 6–10.**

```
cd ../STServo_Python/stservo-env/sms_sts
uv run calibration.py
```

`calibration.py` writes the protection limits to motors 6–10, runs sensorless homing on the prismatic carriage (motor 6) and range-tests all five motors. Homing works like a 3D printer finding its axis limits: the carriage drives in one direction until the motor current spikes against the end stop, records that position, then does the same in the other direction. Details are in the `calibration.py` section.

**Do not unplug or power-cycle the follower board after this step.** The torque settings for motors 6–10 are stored in the servos' volatile memory and are lost when power is cut. If power is lost, run `calibration.py` again.

**If you are going to run script 07, this step can be skipped:** script 07 writes the same protection settings and re-homes motor 6 itself every time it starts. Run `calibration.py` on its own when you only want to home and range-test motors 6–10.

**Step 5: go back to the LeRobot folder** (`cd ../../../so101_lerobot`). The teleop scripts use that folder's own Python environment (`.venv`) and import `so101_follower_5dof.py` from it.

**Step 6: teleoperate.** Pick one:

- `06_teleoperate_base_arm.py`: the leader arm controls only the follower's five arm joints. The carriage and instrument are not controlled.
- `07_teleoperate_full_10dof.py`: the leader arm controls motors 1–5 and the keyboard jogs motors 6–10. It also re-homes motor 6 on every start.

```
uv run scripts/07_teleoperate_full_10dof.py \
    --leader-port <leader-port> --leader-id leader_arm \
    --follower-port <follower-port> --follower-id follower_arm
```

**Note:** if a connection fails with an empty motor list, `scripts/scan_motors.py <port>` pings every ID at every baud rate to check that the motors are powered and wired.

### Debugging connections: `scripts/scan_motors.py`

**Purpose:** `scan_motors.py` checks which motors the bus can actually see. Run it first when a teleop or calibration script fails, especially with an error like `found motor list: {}`. Sometimes the motors stop responding entirely, and this tells you whether the problem is the connection or the code.

**How to run:**

```
uv run scripts/scan_motors.py <port>
```

It needs no calibration file. Stop any other script using the port first, because only one program can hold it at a time.

**What it does:** it pings every servo ID (0–252) at every common FeeTech baud rate (1 Mbps down to 9600) and prints each motor that answers, with its ID and model number. It exits with an error if nothing responds at all. Scanning every rate takes a while.

**Which scan script to use:** three scripts check for motors. `scan_motors.py` is the most comprehensive: it finds every motor present on the bus and the baud rate each is set to. `sms_sts/scan.py` pings every ID but only at 1 Mbps, and `sms_sts/ping.py` checks a single ID you set at the top of the file. Use `scan_motors.py`.

**How to read the result:**

- **Nothing found:** check the servo power supply is on, the daisy-chain cables are seated, and the port is correct.
- **Some IDs missing:** the chain is broken after the last motor that answered. Check that motor's outgoing cable.
- **A motor at the wrong baud rate:** every motor on this robot should be at 1 Mbps (1,000,000).
- **A missing or duplicated ID:** two motors with the same ID corrupt each other's replies. A new servo ships with ID 1 and must be re-numbered before it is added to the chain.

**Servo ID map.** When building the robot from scratch, every servo must be renumbered to the ID below before it goes on the bus, because all new servos ship as ID 1. Number one motor at a time with only that motor connected.

| Arm | ID | Motor name in code | URDF joint | What it moves | Set ID with |
| --- | --- | --- | --- | --- | --- |
| Leader | 1 | shoulder\_pan | — | Base rotation | `02_setup_motors_leader.sh` |
| Leader | 2 | shoulder\_lift | — | Shoulder | `02_setup_motors_leader.sh` |
| Leader | 3 | elbow\_flex | — | Elbow | `02_setup_motors_leader.sh` |
| Leader | 4 | wrist\_flex | — | Wrist pitch | `02_setup_motors_leader.sh` |
| Leader | 5 | wrist\_roll | — | Wrist roll | `02_setup_motors_leader.sh` |
| Leader | 6 | gripper | — | Leader gripper (read but not used) | `02_setup_motors_leader.sh` |
| Follower | 1 | shoulder\_pan | `shoulder_pan` | Base rotation | `04_setup_motors_follower.py` |
| Follower | 2 | shoulder\_lift | `shoulder_lift` | Shoulder | `04_setup_motors_follower.py` |
| Follower | 3 | elbow\_flex | `elbow_flex` | Elbow | `04_setup_motors_follower.py` |
| Follower | 4 | wrist\_flex | `wrist_flex` | Wrist pitch | `04_setup_motors_follower.py` |
| Follower | 5 | wrist\_roll | `wrist_to_rail` | Rail roll (sign inverted vs. URDF) | `04_setup_motors_follower.py` |
| Follower | 6 | prismatic | `carriage_prismatic_joint` | Carriage in/out along the rail | `change_id.py` |
| Follower | 7 | rotary\_7 / wrist\_pitch | `wrist_pitch` | Instrument disk: wrist pitch | `change_id.py` |
| Follower | 8 | rotary\_8 / roll | `roll` | Instrument disk: shaft roll | `change_id.py` |
| Follower | 9 | rotary\_9 / jaw\_1 | `jaw_1` | Instrument disk: jaw 1 | `change_id.py` |
| Follower | 10 | rotary\_10 / jaw\_2 | `jaw_2` | Instrument disk: jaw 2 | `change_id.py` |

The disk assignments for IDs 7–10 come from `simHardware.py`. Script 07 calls them `rotary_7` to `rotary_10` without naming the joint, so confirm each disk's function on the hardware. All motors run at 1 Mbps. `change_id.py` changes one ID per run: set `SCS_ID` (current ID) and `NEW_ID` at the top of the file.

### `scripts/06_teleoperate_base_arm.py` — teleoperate the SO-101 base arm

**This is the actual teleoperation script for the SO-101 base arm.** Moving the leader arm controls the follower arm's motors 1–5. Motors 6–10 (carriage and instrument disks) are not touched.

It lives in the `scripts` folder; run it from inside `so101_lerobot/` like the other scripts. `so101_follower_5dof.py` is the follower class this script uses; it is never run on its own.

The leader reports 6 motors including its gripper, but the follower class only has motors 1–5, so the gripper value is dropped before sending. The loop runs at 60 Hz by default (`--fps`). Ctrl+C stops it.

Usage: `uv run scripts/06_teleoperate_base_arm.py --leader-port <leader-port> --leader-id leader_arm --follower-port <follower-port> --follower-id follower_arm`. The IDs must match the ones used for calibration (steps 2 and 3 of `commands.md`).

### `scripts/07_teleoperate_full_10dof.py` — teleoperate all 10 motors (last fully working script)

**What it does:** the same as script 06, plus keyboard control of the carriage and instrument. It first runs sensorless homing on the prismatic carriage (motor 6). Once homing is complete, the robot is open for use:

- **Motors 1–5 (SO-101 base arm)** follow the SO-101 leader arm.
- **Motor 6 (prismatic carriage) and motors 7–10 (da Vinci instrument disks)** are jogged with held keyboard keys.

#### Keyboard controls

| Motor | Controls | Positive key | Negative key |
| --- | --- | --- | --- |
| 6 | Prismatic carriage | Up arrow | Down arrow |
| 7 | Instrument disk | E | Q |
| 8 | Instrument disk | P | I |
| 9 | Instrument disk | C | Z |
| 10 | Instrument disk | M | B |

ESC or Ctrl+C exits.

#### Details

- **One shared bus.** Motors 1–10 are daisy-chained on one serial port, so the script builds a single LeRobot bus for all 10 rather than opening the port twice, which would corrupt communication. Motors 1–5 use their LeRobot calibration; motors 6–10 are driven in raw encoder steps.
- **Homing every start.** It contains its own copy of `calibration.py`'s homing and protection settings, so motor 6 drives into both mechanical stops each time it starts. Running `calibration.py` beforehand (as `commands.md` lists) is redundant.
- **5-second lockout.** After homing, all 10 motors ignore input for 5 seconds before control goes live.
- **Safe exit.** On exit, motors 7–10 return to their calibrated center (2048) before torque is turned off.
- **Torque.** Motors 7–10 run with a 12.5% torque limit (`ROTARY_TORQUE_LIMIT = 125`).

**Requires:** script 05's follower calibration file, and the motor 7–10 offsets stored in EEPROM by `positionCalibration.py`.

**Check the ports.** The docstring's usage example has the leader and follower ports swapped compared with `commands.md`. Confirm which port is which (`scan_motors.py`) before running.

Usage: `uv run scripts/07_teleoperate_full_10dof.py --leader-port <leader-port> --leader-id leader_arm --follower-port <follower-port> --follower-id follower_arm`. The IDs must match the ones used for calibration.

## `STServo_Python/`: Register-Level Control of Motors 6–10

Scripts that talk to the servos directly through FeeTech's SDK, one register at a time. They handle what LeRobot does not: homing the carriage, centering the instrument disks, torque testing and small utilities. All of them are in `STServo_Python/stservo-env/sms_sts/`.

### Setup: `STServo_Python` environment (for the `sms_sts` scripts)

All the `sms_sts` scripts run from this environment, separate from `so101_lerobot`. Paths and port names below are from the laptop used for this project and may vary from project to project or laptop to laptop.

1. **Go to the folder and create the virtual environment** (first time only):

   ```
   cd ~/Desktop/A*STAR/Ocellivision\ Robotics\ Project/STServo_Python
   uv venv
   ```
2. **Activate it and move into the scripts folder:**

   ```
   source .venv/bin/activate
   cd stservo-env/sms_sts
   ```
3. **Install the requirements** (first time only):

   ```
   uv add -r requirements.txt
   ```
4. **Find the USB port of each arm's servo board:**

   ```
   ls /dev/tty* | grep usb
   ```

   On this project's laptop: leader `/dev/tty.usbmodem5AAF2189541`, follower `/dev/tty.usbmodem5AAF2197651`.

**Update `DEVICENAME`.** The follower port above is hard-coded as `DEVICENAME` in every `sms_sts` script. Change it to your follower port before running anything.

### Reference: `ST3215 memory register map-EN.xls`

**Purpose:** this spreadsheet is FeeTech's register map for the STS3215 servo. It lists every memory address, what it controls, its size in bytes, its initial value, whether it is stored in EEPROM (kept after power-off) or SRAM (reset at power-on), its minimum and maximum, and its **unit (step increment)**. Every register-level setting in this project traces back to it.

**Units matter most.** The same number means different things in different registers, so always check the unit before writing a value. The registers this project uses:

| Address | Register | Memory | Unit / step | Notes |
| --- | --- | --- | --- | --- |
| 9 / 11 | Min / Max angle limit | EEPROM | 1 step (4096 per revolution) | Both set to 0 = multi-turn mode (used for motor 6) |
| 16 | Max torque | EEPROM | 0.1% (1000 = 100% stall torque) | Copied into Torque limit at power-on |
| 18 | Phase | EEPROM | Bit flags | Bit 4 must be set for multi-turn position feedback |
| 30 | Angle resolution | EEPROM | Multiplier | Listed range 1–3 |
| 31 | Position correction (offset) | EEPROM | 1 step | Sign is bit 11, not a normal negative number; ±2047 |
| 40 | Torque enable | SRAM | 0 / 1 / 128 | 128 resets the current position to 2048 |
| 48 | Torque limit | SRAM | 0.1% (1000 = 100%) | The live torque cap; e.g. 125 = 12.5% |
| 56 | Present position | SRAM | 1 step | Read only |
| 69 | Present current | SRAM | 6.5 mA | Read only; multiply the raw value by 6.5 |
| 13–15 | Max temperature / max and min voltage | EEPROM | 1 °C / 0.1 V | e.g. 130 = 13.0 V |

EEPROM registers can only be written after unlocking (address 55 = 0) and should be locked again afterwards (address 55 = 1).

**The listed limits are not always the real limits.** In practice, values outside the listed minimum and maximum can sometimes be written. The servo may then store something different from what you sent, or behave unexpectedly. Always read a register back after writing it to confirm the stored value is what you intended. `Motor.getSetting()` in `motor.py` does this for one register; `printSettings.py` dumps the whole table for one motor.

### `bus.py`

**Purpose:** the `Bus` class manages every motor on one serial port at once. All motors share one port, baud rate, `PortHandler` and `sms_sts` packet handler, so the bus owns those and hands the packet handler to each motor.

**Folder layout note:** the project's own scripts live in `sms_sts/`, next to the vendor SDK folder `scservo_sdk/` one level up. Every script starts with `sys.path.append("..")` so `from scservo_sdk import *` finds the SDK. This is why the scripts must be run from inside `sms_sts/`. The OS-detection block (`getch()` for Windows vs. Mac/Linux) is boilerplate copied from FeeTech's examples.

**What it provides:**

- `openPort()` / `closePort()`, plus `with Bus(...) as bus:` support, so the port always closes when a script ends or crashes.
- `addMotor(...)`: creates a `Motor` (see `motor.py`) already connected to this bus's packet handler.
- `moveMotorsToPositions(motors, moves)`: sends every motor's goal position, speed and acceleration in one sync-write packet so they start moving at the same time, then polls each motor until all have arrived. `moves` is a dictionary `{id: (goalPosition, speed, acceleration)}`.
- `printAllSettings()`: prints the port and baud rate.

**Motor attributes passed through `addMotor`:**

| Attribute | Meaning |
| --- | --- |
| `rawCenterPosition` | The raw encoder reading (before any offset) where the joint is physically centered. Calibration writes an offset so this position then reads as 2048. Example: motor 7's raw center is 2750. |
| `range` | How far the motor may move either side of center, in steps (4096 = one revolution). |
| `minPosition` / `maxPosition` | Software limits, usually center − range and center + range. |
| `positionTolerance` | How close (in steps) counts as "arrived". |
| `minPositionLimit` / `maxPositionLimit` | Hardware limits written to the servo's EEPROM, usually `margin` steps outside the software limits as a backup. |
| `positionMargin` | The gap between the software and hardware limits. |
| `angleResolution` | Scales the size of each position step: at 1 a step is 0.088°; at 2 or 3 each step moves 2× or 3× as far, extending the commandable range. The register map lists 1–3 as valid. |

**Caution:** `moveMotorsToPositions` has no timeout. If a motor stalls or stops responding, the script waits forever; stop it with Ctrl+C.

### `motor.py`

**Purpose:** the `Motor` class controls one servo. It stores that motor's limits and settings and wraps the SDK calls for reading, writing and moving it. Motors are normally created through `Bus.addMotor()`. Each attribute is explained in comments in the constructor.

**Raw center vs. calibrated center:** the calibrated center is always 2048. The raw center is the motor's original, uncalibrated reading at the point where the two holes in its custom motor horn are exactly horizontal (for example, 2750 for motor 7). An offset is calculated from the raw center so that position reads as 2048 afterwards.

**Why the limits matter:** the min and max positions are most important for motors 7–10, which drive the instrument's wrist pitch, roll and jaws through cables. Rotating past the safe range can snap the cables or break the instrument.

**Methods:**

| Method | What it does |
| --- | --- |
| `getSetting` / `setSetting` | Read or write any register by address. Use `getSetting` after `setSetting` to confirm the value was stored. `getSetting` returns `None` if the read fails. |
| `unlockEeprom` / `lockEeprom` | EEPROM registers (addresses below 40) can only be written while unlocked. Lock again afterwards. |
| `getPosition` / `getPositionSpeed` | Read the present position (and speed). The servo applies its offset before reporting, so this is the calibrated position. |
| `setPosition` | Sends a goal position with speed and acceleration, then returns immediately without waiting. |
| `moveToPosition` | Sends a goal position, then waits until the motor arrives (or is already within `positionTolerance`). Use this when the next step must wait for the move to finish. |
| `isMoving` | Reads the servo's moving flag. Used to detect arrival. |
| `getTelemetry` | One read that returns position, speed, load, voltage, temperature, moving flag and current (mA). Used for homing and torque tests. |
| `applyPositionLimits` | Writes the hardware min/max angle limits to EEPROM. |
| `applyAngleResolution` | Writes the angle resolution and sets the multi-turn bit in the Phase register. |
| `printAllSettings` | Prints the stored attributes and the present position. |

**How "arrived" is detected (used in every move loop in the project):** right after a goal is sent, the servo can still report "not moving" because it has not started yet. Checking only the moving flag would end the wait instantly. So the loop keeps a `hasMoved` flag: it is set once the servo reports moving, and the move counts as finished only when the servo then reports stopped. The one exception is a goal already within `positionTolerance` (the servo's dead zone), where the servo may never start moving at all; that also counts as arrived.

**Known issues:**

- The constructor writes to the servo whenever `angleResolution` is not 1. Creating a motor with `angleResolution=0` or `None` therefore changes its EEPROM or crashes. This is the cause of the `printSettings.py` and `positionCalibrationOOP.py` bugs. The fix is to remove that write from the constructor and call `applyAngleResolution()` explicitly when needed.
- `setSetting(..., signed=True)` encodes negative numbers with the sign in bit 15. That is correct for goal position, but the position offset register (31) uses bit 11, so negative offsets need different handling.
- `moveToPosition` has no timeout.

### `telemetry.py`

**Purpose:** the `Telemetry` class draws a live plot of a motor's current draw (mA) over time while a script runs, with labelled vertical lines marking each phase (for example "Obstacle Search #1"). It makes current spikes and the safe operating level visible as they happen.

**Why current, not torque:** the servo cannot measure the torque it produces, so current is used as a stand-in. Motor current rises with the torque the motor is exerting, so high current means the motor is straining against something. The servo also reports a "Present Load" value, but that is the PWM duty cycle driving the motor, not a torque measurement.

**Why it exists:** during early testing, a disk motor ran with too much torque and snapped one of the da Vinci instrument's cables, making that instrument unusable. Monitoring current, together with the torque characterization test, was introduced so torque and current limits for the fragile instrument motors (7–10) could be set from measured data.

**Where it is used:** `calibration.py` and `prismaticJointCalibration.py` use it during sensorless homing of motor 6. Dotted lines on the plot mark the spike threshold, so you can see the moment the carriage hits an end stop. `torque_characterization_test.py` does not use this class; it has its own plotting function and also saves CSV files.

**Methods:** `sample(currentMa)` adds a point and redraws; `markEvent(label)` adds a phase marker; `freeze()` then `show()` keep the final plot open when the run ends; `savefig(path)` saves it. Data is kept in memory only, not saved to a file.

**Notes:** it needs a display and `matplotlib`. Redrawing on every sample is slow, so it limits how fast the homing loop can poll.

### `calibration.py` — sensorless homing of the carriage, then a range test of all five motors

**What it does:** sets protection settings on motors 6–10, homes the prismatic carriage (motor 6) by finding both ends of its travel, drives it to the center, then range-tests motors 6–10. It replaces `prismaticJointCalibration.py`, which homed motor 6 only. Script 07 contains its own copy of this homing logic.

**Why sensorless homing:** the carriage has no limit switches. The idea comes from 3D printers' sensorless homing: drive the axis until it hits its mechanical stop, and detect the stop from a spike in motor current instead of from a switch.

#### Settings

**Motor 6 (prismatic carriage)**

| Setting | Meaning |
| --- | --- |
| `PRISMATIC_ID` | ID of the prismatic motor (6). |
| `homingSpeed`, `homingAcc` | Speed and acceleration used while searching for the ends and driving to center. |
| `movingSpeed`, `movingAcc` | Faster speed and acceleration used after homing, for the range test and later movement. |
| `RANGE_BUFFER` | Steps kept back from each detected end during the range test, so the carriage never runs into the stops at full speed. |
| Back-off | Distance (about half a revolution) the carriage reverses immediately after detecting an end. See *Why the back-off matters* below. |
| `SCS_MOVE_TOLERANCE` | Steps within which a move counts as arrived. |
| `SEARCH_DISTANCE` | 20,000 steps: an arbitrarily large target for each search leg. The servo needs a goal position, and this is far enough that the carriage always hits a stop first. |
| `SAMPLE_HZ` | Telemetry sampling rate for the live current plot. |
| `SPIKE_CURRENT_THRESHOLD_MA` | 35. Current above this means the carriage has hit an end. It is in **real milliamps**: the Present Current register counts in 6.5 mA units, but `Motor.getTelemetry()` multiplies by 6.5 before the comparison. |
| `STOP_SETTLE_TIMEOUT_S` | Up to 2 s to wait for the motor to report it has stopped after a spike. |
| `SETTLE_DWELL_S` | An extra 0.3 s after stopping so leftover current dies down before the next move. |
| `HOMING_TORQUE_LIMIT` | 125 = 12.5% of max torque, so the carriage only taps the ends gently. |
| `movingTorqueLimit` | 400 = 40%, restored after homing for the range test and normal movement. |

Protection settings (max temperature, voltage limits, unloading conditions, LED alarms, dead zones) are applied to every motor. Their register addresses are defined separately from the values written to them.

**Motors 7–10 (instrument disks)**

- `SCS_CALIBRATED_CENTER` = 2048, each motor's true center after offset correction. The comment says the offsets come from `positionCalibrationOOP.py`, but that script crashes on startup; they come from **`positionCalibration.py`**.
- `ROTARY_TORQUE_LIMIT` = 125 (12.5%) for their range test.
- `MOTOR_CONFIGS` gives each motor's range, speed, acceleration, tolerance and margin.
- **Software vs. hardware limits:** the software limits are center ± range. The hardware limits written to the motor are the software limits widened by `margin` on each side, so they only act as a backup if the software limit fails.

**Multi-turn tip:** motor 6 has both hardware angle limits set to 0. That gives unrestricted multi-turn rotation with no clipping at ±4095, and it works better than changing Angle Resolution. **For multi-turn, set both position limits to 0 rather than changing Angle Resolution.**

#### Sequence

1. Keep Angle Resolution at 1 and set the multi-turn Phase bit.
2. Set motor 6's position limits to 0/0 (unrestricted travel).
3. Reset motor 6's position correction to 0 and lock the EEPROM.
4. Cap motor 6's torque at 12.5% and enable torque.
5. Read the start position and open the telemetry plot.
6. **Search forward:** drive toward start + 20,000 until current crosses 35 mA. Stop, settle, and back off. Record the forward end.
7. **Search backward** the same way. Record the back end.
8. Compute the center as the midpoint of the two ends, print all three, and drive to the center. Homing is complete.
9. Raise motor 6 to its runtime torque limit (40%) and cap motors 7–10 at 12.5%.
10. Set motor 6's range-test limits to each end minus `RANGE_BUFFER`.
11. **Range test:** each motor in turn goes to minimum, center, maximum, then back to center.

#### Why the back-off matters

When the carriage presses into an end, current climbs well above the threshold, say to 60 mA. If the script simply reversed, current would still be above 35 mA as the carriage eased off the stop, and the script would immediately mistake that for the second end, recording both ends at the same position. Backing off about half a revolution right after each detection lets current drop below the threshold first, so it only spikes again at the real opposite end.

#### Stopping the program

When it finishes, or if homing fails, the telemetry plot stays open. **Close the plot window to end the program**; that is what lets the script exit. If homing fails or you press Ctrl+C, torque on motor 6 is turned off.

### `positionCalibration.py` — defines where center is

**This is the recalibration and recovery script.** Run it before starting work if the motor horns on motors 7–10 (the motors that drive the da Vinci instrument's joints) are not at their level position, for example after an emergency stop. It re-establishes each motor's center and returns the motors there. Its final sweep also works as a range test.

**Why each motor needs its own offset:** the custom 3D-printed motor horns all sit at slightly different angles on their servos. Center is defined as the position where a horn's two socket holes are exactly horizontal, so each motor has its own raw center:

| Motor | Raw center | Range from center |
| --- | --- | --- |
| 7 | 2750 | ±700 |
| 8 | 1750 | ±700 |
| 9 | 3385 | ±1400 |
| 10 | 3860 | ±1400 |

**Steps:**

1. Move motors 7–10 together to their *desired center raw*: the raw position where the horn's two holes are level, which should become 2048.
2. Calculate each offset (desired center raw − 2048) and write it to register 31 (Position Correction) in EEPROM.
3. Read register 31 back to verify the write.
4. Move all motors to the newly calibrated center and confirm each reads 2048.
5. Range-of-motion check: move all motors **simultaneously** to minimum → center → maximum → center, so they finish centered.

**Use this non-OOP version.** `positionCalibrationOOP.py` is broken: it builds its motors before the port is open and passes `angleResolution=None`, so it raises an exception on startup.

**Check motor 8's offset.** Its correction is negative (−298). Register 31 stores the sign in bit 11 (range ±2047), but this script writes a plain integer, so the stored value may not be −298. Read register 31 back on motor 8 before trusting its center.

### `rangeTest.py` — range check when center is already correct

**Use it to see each motor's range when the center is already correctly established.** It does not correct the center and saves nothing to the motors; it relies on the offsets `positionCalibration.py` stored. If the horns are not level, run `positionCalibration.py` instead.

For each motor 7–10 in turn, it moves to its minimum, center (2048), maximum, then back to center. Every motor therefore ends centered.

**Warning:** it sweeps each motor to both limits before centering, from wherever the motor currently is. Make sure the instrument is free to move first.

**In short:** `positionCalibration.py` re-centers the motors and range-tests them; `rangeTest.py` only range-tests around a center that is already correct. **Do not use the OOP versions** (`positionCalibrationOOP.py`, `rangeTestOOP.py`); use these two.

### `stopMotor.py` — cut torque on one motor

Set `SCS_ID` at the top (currently 6) and run it. It turns torque off on that motor, so the motor stops and can be moved by hand, then prints its raw position.

Cutting torque is the safest immediate stop: it works even if the motor's last goal position was bad, and doesn't depend on knowing where the motor is. Use it to free a motor that is stuck straining after an emergency stop. Afterwards, run `positionCalibration.py` to bring motors 7–10 back to center, or `calibration.py` to re-home the carriage.

### `printSettings.py` — dump every register on one motor

Connects to the motor whose ID is set at the top (currently 9) and prints every register in the control table: the EEPROM settings, which persist across power cycles, then the SRAM values, which reset on power-up. Useful for checking what a motor is actually configured to before running anything else.

**⚠️ Do not run it until this is fixed.** It creates its motor with `angleResolution=0`. The `Motor` constructor writes any Angle Resolution other than 1 to the motor's EEPROM, so running this script writes Angle Resolution 0 and sets the Phase multi-turn bit on the motor it reads. **Fix:** change `angleResolution=0` to `angleResolution=1`.

### `change_id.py`

**Purpose:** changes one servo's ID. Used to number motors 6–10 (see the servo ID map). It is FeeTech's example script, edited for this project.

**How to use:**

1. Connect only the motor being renumbered, so no other servo shares its current ID.
2. Set `SCS_ID` (its current ID; new servos ship as 1) and `NEW_ID` at the top of the file, and check `DEVICENAME` matches your port.
3. Run `python change_id.py` from inside `sms_sts/`. It unlocks the EEPROM, writes the new ID and locks it again.
4. Confirm with `scan_motors.py` that the motor now answers on its new ID.

### `torque_characterization_test.py`

**Why it exists:** during early testing of the instrument motors, the two jaws were pressed into each other too hard and one of the instrument's cables snapped, making that da Vinci instrument unusable. This showed that torque and safety limits on motors 7–10 are critical. The test measures how much current and load a disk motor uses under different loads and torque limits, so safe limits can be chosen from data instead of guessed.

**Inspiration:** the idea comes from the Golgi tendon organ in kinesiology. It senses tension in a muscle's tendon, and when the load becomes dangerously high (for example, an overly heavy bicep curl), it inhibits the muscle so the weight is dropped before the muscle or tendon is injured. The servo equivalent is a torque cap plus an automatic stop when current gets too high.

**Test method (two phases):**

1. **Increase the load, keep torque fixed** (like finding a one-rep max). Screws were hung from the gripper with wire ties, and the mass was increased step by step to find what the motor could carry. Even with several screws (up to 48.1 g), it kept lifting without trouble.
2. **Keep the load fixed, decrease torque.** With 48.1 g as the fixed test mass, the torque limit was lowered step by step to find the lowest torque that still moves the load. The working limit is then set slightly above that minimum, so the motor can do its job but cannot pull hard enough to snap a cable.

During each run the instrument is positioned so the tested joint swings up and down in a vertical plane, like a bicep curl, lifting the hanging mass against gravity.

**Results (`torque_test_data/`):**

| Phase | Mass | Torque limit | Peak current | Full travel? |
| --- | --- | --- | --- | --- |
| 1 | 0.8, 10.1, 19.4, 28.7, 38, 48.1 g | 25% | 32 mA for every mass | Yes |
| 2 | 48.1 g | 25% | 32 mA | Yes |
| 2 | 48.1 g | 20% | 26 mA | Yes |
| 2 | 48.1 g | 15% | 20 mA | Yes |
| 2 | 48.1 g | 12.5% | 20 mA | Yes |
| 2 | 48.1 g | 10% | 13–20 mA | Yes, but slower (about 1.7× longer than at 25%) |

**What the data shows:**

- Every setting down to 10% still completed the full motion with 48.1 g. The lowest working torque was not reached, so it is below 10%.
- The measured load always sat at exactly the torque limit, and changing the mass made no measurable difference to current. Friction in the cables and instrument, not the hanging mass, dominates the load. Lowering the limit only slowed the motion.
- The project settled on 12.5% for motors 7–10: 10% worked but was noticeably slow, so the limit was set slightly above it. The torque limit register uses 0.1% units, so 12.5% is written as 125 (`ROTARY_TORQUE_LIMIT = 125`); writing 12 would mean 1.2%. This is above the 3–5% starting point recommended for EndoWrist cables, so testing lower limits (5–10%) with a heavier test mass is a worthwhile next step.

**How the code works:** this script was written before the `Bus` and `Motor` classes, so it has its own copies of the setting and move helpers.

- `move_to_position`: the standard move-and-wait loop (see "How arrived is detected" in the `motor.py` section).
- `move_to_position_with_logging`: the same arrival check, but on every poll it reads full telemetry in one combined read and logs it, instead of only printing position and speed. Each sample is written to the CSV and kept for the end-of-run plot and summary.
- `hold_with_logging`: holds the motor still at a position for `HOLD_SECONDS` (2 s) while logging. The holds happen at each end of travel. The most demanding hold is where the joint is horizontal (90° from vertical), because the hanging mass then pulls perpendicular to the gripper and exerts the most torque.

**Telemetry sampling:**

- `read_telemetry`: the present-value registers (addresses 56–70) sit next to each other in memory, so one 15-byte read returns everything at once: position, speed, load (% of max), voltage, temperature, status, moving flag and current. Current is the raw value × 6.5 mA. One read instead of seven is what makes steady 30 Hz sampling possible.
- `sample_and_log`: takes one reading, writes it as a row to the CSV (17 columns, including phase, cycle, goal position and test mass), saves it for the plot, and prints it. It is also the safety stop: if current goes above `DEFAULT_ABORT_CURRENT_MA` (900 mA), or the servo reports a voltage, overheat, overcurrent or overload fault, it raises `AbortExperiment` and the run ends safely.
- Each loop sleeps for whatever is left of its 1/30 s period, so samples are evenly spaced.

**Settings (top of the file):**

- `BAUDRATE` (1,000,000): the serial communication speed between the computer and the servos. `DEVICENAME`: the serial port path for the bus (e.g. `/dev/ttyACM0`); change it to match your machine.
- `SCS_CALIBRATED_CENTER` (2048): the position every motor reads at its center after offset calibration.
- `HARD_ANGLE_LIMIT_MARGIN` (100): how many steps beyond the software min/max the hardware angle limits are written, so the servo itself stops the motor if the software limit ever fails.
- `MOTORS`: the empirically measured settings for motors 7–10: `desired_center_raw` (raw reading that becomes 2048 after calibration), `range` (steps the motor may move either side of center, not the total span), and a deliberately low speed and acceleration to protect the fragile instrument. Correction, min and max positions are calculated from these.
- `ACTIVE_TEST_ID` (9): the only motor driven in the test; motor 9 is one of the jaws. `SCS_POSITION_TOLERANCE` (3): within ±3 steps counts as arrived.
- `NUM_CYCLES` (min-to-max cycles per run), `HOLD_SECONDS` (pause at each end), `SAMPLE_HZ` (30).
- `TEST_TORQUE_LIMIT` (125 = 12.5%; the comment saying 25% is outdated) and `DEFAULT_ABORT_CURRENT_MA` (900).
- Protection settings written to all four motors: max temperature (55 °C), max/min input voltage (13.0 V / 9.0 V), dead zones, and hardware angle limits. The unloading condition (register 19) chooses which faults make the servo cut its torque: voltage, sensor, temperature, current, angle or overload. The LED alarm condition (register 20) chooses which faults flash the servo's LED. Both use one bit per fault (see the register map).

**Run sequence:**

1. Safety settings are written to motors 7–10 and read back to verify.
2. A safety reminder is printed; type `yes` to continue.
3. Enter the hanging test mass in grams (used in the CSV name and plot title).
4. Motor 8 (the other jaw, which shares the instrument with motor 9) is moved out of the way to its maximum position. The code comment says minimum; the code uses maximum.
5. A CSV is opened in `torque_test_data/`, named with the mass, torque limit and timestamp, and motor 9's runtime torque limit is set.
6. For each cycle, motor 9 moves to its minimum, holds, moves to its maximum, holds, then returns to center. Every phase is logged at 30 Hz, and the run aborts if current or a fault trips the safety stop.
7. At the end (or on abort), a summary of mean and peak current and load per phase is printed, the two-panel plot (load and current over time) is shown and saved, and motor 8 is returned to center.

**Two layers of angle limits:** every instrument motor has a software limit and a hardware limit. For motor 9, the software limit is ±1400 steps from center: the code never commands a position beyond it. The hardware limit, written to the servo's EEPROM, sits 100 steps further out (±1500). If the software ever sends a bad position, the servo itself refuses to go past the hardware limit, protecting the cables.

**Check before running:** the saved data used test travel of ±800 steps. The test moves between `min_position` and `max_position`, which come from `MOTORS[9]["range"]`, and that is now 1400. Running the file as it stands moves the jaw ±1400 under load. To repeat the original test, add a separate test-travel setting of 800 instead of changing the motor's range.

## `ps4_teleop/`: PS4 Controller and Simulation

**Copies:** `01_find_ports.sh`, `05_calibrate_follower.py` and `so101_follower_5dof.py` in this folder are copies of the versions in `so101_lerobot`. Only their file paths were adjusted so they work in this flat folder with its own environment; the code is otherwise the same.

### `controller_input.py` — stage 1: read the PS4 controller

This only proves the controller is read correctly. It drives nothing: no IK, no joint targets, no motor output, and no handbrake or e-stop yet.

- It polls for a connected controller. If none is found, the window prompts you to connect one over Bluetooth and keeps polling, so the controller does not need to be connected before starting.
- Once connected, it continuously shows every currently active button and axis alongside its planned robot-control meaning, or "Waiting for input..." when idle.
- A **raw debug panel** is always shown, listing every currently pressed button index and every axis away from its rest value, unfiltered by the guessed mapping. Use it to find and fix any wrong index in `BUTTON_MAP`, which is only a first guess.

**Setup and run:** `uv sync`, then `uv run controller_input.py`.

#### Planned controls

| Input | Planned action |
| --- | --- |
| Right stick | Move in X and Y |
| L2 / R2 | Move −Z / +Z |
| Left stick | Pitch and yaw |
| L1 / R1 | Roll left / right |
| Circle / Square | Right jaw open / close |
| D-pad left / right | Left jaw open / close |
| Cross | RCM mode (not yet implemented) |
| Triangle | Return to home (not yet implemented) |
| Options | Show current keybinds |

**Competing inputs:** some opposite controls can be held at once (L1/R1, Circle/Square, L2/R2). Whichever is pressed first is the one used, until it is released.

#### Next step: stop moving when the input is released

When this is wired to the robot, holding an input (for example R2 for +Z) will keep moving the goal position. If the goal moves faster than the robot can follow, the robot keeps moving after the input is released. It should stop as soon as the input is let go. Two possible fixes:

1. **Check the input is still held.** Only move while the input is pressed; on release, set the goal position to wherever the robot currently is.
2. **Step only when the last goal is reached.** Increment the goal position only once the previous goal has been reached.

### `pastProject.py` — the reference for the simulation logic

**Read this before working on `simv2.py` or `simHardware.py`.** It is a script from a previous internship, written for UR5 robots in Isaac Sim. It is high-quality, extensive code, and `simv2.py` is built on its logic: how targets are read from the inputs, scaled, turned into TCP poses and solved with IK to make the robot move is essentially the same. When something in `simv2.py` is unclear, look at how `pastProject.py` does it.

### `simv2.py` — teleoperate the full robot in simulation (replaces `sim.py`)

Loads the merged URDF into a PyBullet GUI and lets you drive the instrument tip (the TCP: the absolute endpoint of the robot, `PSM1_tool_tip_link`) in Cartesian space from the keyboard. Every loop it solves IK for all 10 movable joints. Nothing is sent to real motors.

#### Threading functions

Two background functions read input and write it into a shared dictionary (`posRotVariables`), guarded by a lock so the main loop never reads it half-updated.

- **`getPygameData`, the one in use.** Reads held keys from a small pygame window. **Click that window so it has focus**, or key presses won't register. Each held key sets a step of 0.001 (metres, or radians for rotation); releasing it sets the step to 0. Key mapping is in the table below.
- **`getPS4Data`, work in progress, commented out.** Reads the L2/R2 triggers for Z only. `DIVIDING_FACTOR` (1000) scales the trigger reading (−1 to 1, rounded to 0.1) down to at most 0.001 m per loop, matching the keyboard step. Not working well yet: both triggers send the same sign, and pygame triggers usually rest at −1 rather than 0, so it can drive Z with nothing pressed.

**Keyboard controls:**

| Key | Moves | Positive | Negative |
| --- | --- | --- | --- |
| Up / Down arrows | X | Up | Down |
| Left / Right arrows | Y | Left | Right |
| Page Down / Page Up | Z | Page Down | Page Up |
| E / Q | Roll | E | Q |
| W / S | Pitch | W | S |
| D / A | Yaw | D | A |

#### Helper functions

- **`getJointPositions`:** gets the positions of all the robot's movable joints.
- **`getSimTCPPose`:** gets the TCP's position and rotation (quaternion) in the simulation.
- **`drawTCPFrame`:** draws the TCP's X, Y and Z axes as a visual reference.
- **`moveRobot`:** moves the TCP to a target pose, described below.

#### `moveRobot`: IK with a reachability check

**PyBullet's biggest problem: it never tells you an IK solution is impossible.** `calculateInverseKinematics` always returns joint angles that get as close as it can, even when the target is out of reach. `moveRobot` checks the result itself:

1. **Save the current joint positions**, to restore if IK fails.
2. **Solve IK** for the target position and rotation, within the joint limits. `jointDamping`, `restPoses`, `jointRanges`, `maxNumIterations` (500) and `residualThreshold` (1e-5) all tune the IK solver; see the PyBullet documentation for what each does.
3. **Test the solution without stepping the simulation.** Each joint is set straight to its IK angle (`resetJointState`), so the robot is in the new pose in the back end without visibly moving there.
4. **Measure the error with forward kinematics.** Read the TCP pose in that new position. Position error is the 3D distance from the target; rotation error comes from the dot product of the two quaternions, converted to degrees.
5. **Decide.** If either error is beyond its tolerance (`POSITION_TOLERANCE_M` = 0.1 m, `ROTATION_TOLERANCE_DEGREES` = 10°, both adjustable), the target was likely out of the robot's positional or rotational reach: print `IK FAILED` and reset the joints to the saved positions. Otherwise, send the IK angles to the joint motors as position targets.

**Known limitations:**

- Because step 3 already placed the robot in the new pose, a successful move is effectively a jump rather than a smooth motion. A real arm cannot do that, so this needs changing before driving hardware.
- 0.1 m (10 cm) is a loose position tolerance for a surgical tool; tighten it once IK is reliable.

#### Simulation setup

- **Gravity is set to 0** because gravity caused problems. Getting it working with gravity is a next step.
- **URDF path is hard-coded** to `/home/user/Desktop/...`. Change it to wherever your URDFs are stored.
- `baseOrientation` sets the orientation of the robot's base.
- **`useFixedBase=1` is important:** it fixes the robot to the ground instead of leaving it a free-floating object.
- `URDF_USE_SELF_COLLISION` lets the robot collide with itself.
- Real-time simulation is off while loading, then turned on when teleop starts.

#### Joint setup

It loops over every joint in the URDF and stores each one's index, type and limits by name. The TCP is found as the joint whose child link is `PSM1_tool_tip_link`. If you need a different TCP link, the URDF viewer (viewer.robotsfan.com) is very useful for finding the right link name.

It then builds the list of 10 joints IK may move: the five arm joints, the carriage's prismatic joint, then roll, wrist pitch and the two jaws. For each, it builds lists of lower limits, upper limits, ranges (upper − lower) and damping (1.0 for every joint), which are passed to the IK.

#### Main loop

1. Start the keyboard thread and draw the TCP frame.
2. Turn on real-time simulation and print the joint positions and TCP pose.
3. Move to the start target, then wait 10 seconds.
4. Loop: read the latest steps through the lock; get the TCP's current pose; add the position step; apply the rotation step on top of the current rotation, so roll, pitch and yaw turn about the tool's own axes; call `moveRobot`.

Because each target is the **current** pose plus a step, the goal never runs ahead of the robot, so it stops when the key is released.

On exit (Ctrl+C) it stops the keyboard thread and closes PyBullet.

**Check the start target:** on start it moves to `[current x, 0, 0]`, which is z = 0 (floor height). Check it before reusing this on hardware.

### `simHardware.py` — simulation driving the real robot (unstable, not working)

**⚠️ This script is unstable. Treat it as a reference and recreate it yourself rather than running it as-is.**

It is essentially `simv2.py`, except that whenever a valid joint solution is found in the simulation, it is also sent to the real robot. This mimics the same logic as `pastProject.py`.

**What it adds to `simv2.py`:**

- **Hardware constants** copied from script 07 (motor names, protection limits, motor 7–10 settings). Script 07 is the source of truth for these; don't retune them here.
- **Arm (motors 1–5):** the IK angles are converted from radians to degrees and sent through the LeRobot calibration. Wrist roll's sign is inverted (confirmed on hardware); the URDF calls this joint `wrist_to_rail`.
- **Instrument (motors 7–10):** radians are converted straight to encoder steps around the calibrated center, 2048.
- **Carriage (motor 6) is deliberately not sent.** It needs the homing logic from script 07 first.

Run from `ps4_teleop/`: `uv run simHardware.py --follower-port <follower-port> --follower-id follower_arm`

**Why it doesn't work:**

1. **The start target is floor height.** It moves to `[current x, 0, 0]` (z = 0) on real hardware, with no speed limit.
2. **Successful IK leaves the simulated robot teleported.** The next target is then calculated from that jumped pose.
3. **No sleep in the control loop,** so it floods the motor bus.
4. **Unverified instrument mapping.** Motors 7–10 use a 1:1 radian-to-radian mapping with unchecked signs, ignoring how the cables couple the instrument joints.
5. **`wrist_yaw` is a fixed joint in the URDF,** so the IK cannot use it.

## Mechanical Design (CAD)

The parts were designed in Fusion 360, and the design files are in the repo: the complete assembly, the prismatic carriage, the motor-to-disk adapter, the motor-to-GT2 adapter and the RCM test stand. Some STL exports include the STS3215 servo model in the mesh, so they are for reference, not direct printing.

### `Motor_to_Disk_Adapter` — custom motor horn for the instrument disks

A custom horn for the STS3215 servos on motors 7–10. It lets each servo drive one of the four input disks on the da Vinci instrument's base. Seen from the top, it has six holes:

- **Four central mounting holes** secure the horn to the servo. They are counterbored at the top and countersunk at the bottom, to suit countersunk screws.
- **An alignment nub** on the underside sits on the servo's central axis, so the horn is centered before the screws go in.
- **Two off-center socket holes** are female sockets for the two male pins on each da Vinci input disk. **They are intentionally off-center**, because the pins on the instrument's disks are off-center. Keep this offset if you redesign the horn.

These two socket holes are also the reference for centering: `positionCalibration.py` turns each motor until its horn's two holes are exactly horizontal and saves that position as 2048. Each printed horn sits at a slightly different angle on its servo, which is why each motor has its own offset.

### `Motor_to_GT2_Adapter` — drive shaft for the carriage belt

Connects motor 6 (the carriage servo) to the GT2 belt that moves the prismatic carriage. It has three parts:

- **Base:** the same as the disk adapter's, with the same four countersunk holes securing it to the servo.
- **Shaft with a 20-tooth GT2 pulley:** a long shaft protrudes from the base and carries the pulley. The shaft has two holes that match the pulley's two grub screws. The grub screws tighten into these holes, which locks the pulley onto the shaft in exactly the right position and stops it slipping.
- **Two ball bearings,** one on each side of the pulley, which carry the belt's load. Size: 22 mm outer diameter, 8 mm inner diameter (the standard 608 size).

**Why the bearings:** the GT2 belt has to be very tight. As the servo turned the pulley to move the carriage, the belt tension pulled the shaft sideways, bending it perpendicular to its axis of rotation (toward the rods). The two bearings are held fixed in the prismatic carriage's frame (see the carriage CAD), so they absorb the belt's side load instead of the shaft and servo taking it.

### `Prismatic_Joint_Carriage` — the prismatic joint

The carriage slides the da Vinci instrument in and out along two **8 mm carbon fibre rods**, driven by motor 6 through a GT2 belt. It has four main parts: the rod back end, the rod front end, the bottom cage and the top cage.

#### Carbon rod back end

A flat piece that acts as the **rear stop**, setting the carriage's backmost limit. The back ends of both carbon rods mount to it. The belt's **idler pulley** runs through its central hole, held by a screw and nut; the belt loops around it at the back, opposite the motor's GT2 pulley.

#### Carbon rod front end

A much chunkier piece with several jobs:

- **Front stop,** with the front ends of both rods mounted to it.
- **Holds motor 6,** which drives the belt through the `Motor_to_GT2_Adapter` (pulley fixed to the shaft by its grub screws, with a 608 bearing on each side).
- **Mounts to the SO-101.** Underneath is a connector identical to the SO-101's own joints, including the small nub that acts as a rotation-limiting stop.

**Bearing cups.** The two bearings press into cups in the front end's body: one cups its bearing from below, the other from above. Because they grip from opposite directions, the bearings can't slip out either way. The PLA around them absorbs the belt's pull, so the thin shaft doesn't have to. The end of the shaft also sits in a small holder that gives the tip light extra support.

**Side entry and dovetail.** The motor, adapter and bearings form one long assembly that can't fit in over, under or through the front opening (the inspiration was *We're Going on a Bear Hunt*: "we can't go over it, we can't go under it, we've got to go through it"). So it goes in through an **access hole in the side**. The hole is partly round so the adapter's round base can pass through. A **dovetail** (`carbon rod front end dovetail`) then slides in from the top and closes the hole behind the motor.

**Assembly order:**

1. Press both bearings into their cups. The fit is between a transition and a light interference fit.
2. Attach the `Motor_to_GT2_Adapter` shaft to the motor.
3. Hold the 20-tooth GT2 pulley in place and slide the motor and shaft in through the side hole, until the shaft end seats in its holder.
4. Slide the dovetail in from the top, and secure the motor to it with the four counterbored screws.
5. Route the motor's wires through the wire access hole.

**For the next version:** the dovetail is a tight interference fit and hard to remove. Loosen it so it comes out more easily.

#### Bottom cage

The most complex part. It holds motors 7–10 and carries the whole carriage along the rods.

- **Motor pockets:** four pockets around the center, with clearance for each motor's screws and protrusions. Each motor sits face up with its `Motor_to_Disk_Adapter` on top. The motors are screwed in from underneath through access holes in the bottom face, which also has wiring ports.
- **Rod bearings:** arched "wings" on each side wrap around the carbon rods. Underneath are four slots for **igus drylin RJMP-01-08** linear bearings (8 mm), two per rod. These are plain, self-lubricating plastic bearings that run dry on the rod, which suits carbon fibre better than ball bearings. Each is held in with zip ties through holes on either side of its slot.
- **Central tower:** a post between the four motors with two jobs:
  1. Its top matches a gap in the underside of the da Vinci instrument's base. It presses up into the instrument and into the top cage, so the instrument can't move in X, Y or Z.
  2. A slit in its side lets the return run of the belt pass through freely, between the motor pulley and the idler.
- **Belt clamps:** at the front and back, the belt lies teeth-up in a sunken lane between two raised mounds. A separate **front clamp** and **back clamp** press down onto it, meshing their tooth profile with the belt's so it can't slip. A screw goes down from the top through each clamp and the bottom cage; a nut comes up from below through an access hole, and tightening them presses the clamp onto the belt. This is how pulling the belt moves the cage.
- **Top-to-bottom connection:** at each corner of the central section, a screw comes down from the top cage, with its head sitting deep in a counterbore, and passes through the bottom cage into a nut that sits flush with the bottom face. Beside each hole, a square socket in the bottom cage takes a square peg on the top cage. These pegs are a **poka-yoke** (mistake-proofing) feature, so the halves only fit together the right way.

**For the next version:** all four corners have a peg, so the top cage can still go on rotated 180°. Put pegs at only three corners so there is just one possible orientation.

#### Top cage

Has an opening the da Vinci instrument slides into from underneath, shaped to the sides of the instrument for a snug fit. At the front, an arch lets the instrument's long shaft out. A **dovetail** (`dovetail top cage`) slides in from the bottom to close the arch into a round outlet around the shaft, so the instrument can't slide back down.

#### Sacrificial bridge layers

Several holes look closed in the CAD (in the front and back clamps, and the deep counterbores in the top cage) but are meant to be through holes. Each has a **sacrificial bridge layer**: a thin skin, one or two layers thick, modelled across the hole where it starts above a counterbore or pocket. It lets the printer bridge cleanly across instead of printing the hole's edges in mid-air. **After printing, push the screw through or drill the skin out** to open the hole.

#### Full carriage assembly order

1. Put the da Vinci instrument into the top cage, then slide the top cage dovetail in.
2. Assemble everything on the bottom cage: motors 7–10 with their disk adapters, the RJMP bearings, the belt and its front and back clamps, and the carbon rods.
3. Last, join the top cage to the bottom cage with the four corner screws.

**Watch out:** joining the two cages can be fiddly, because the four disk adapters' sockets have to line up with the pins on the instrument's disks at the same time as the cages line up.

### Complete assembly — SO-101, prismatic carriage and da Vinci instrument

The whole robot together:

- **SO-101 base arm:** its five motors (1–5), with the gripper removed.
- **Prismatic carriage:** the carbon rod front end mounts onto the top of the SO-101 arm, in place of the gripper, through its SO-101-style connector.
- **da Vinci instrument:** slides into the carriage's cages.

**Assembly order:** screw the carbon rod front end onto the SO-101 arm **first, before putting any motors into it**. Then follow the carriage assembly order above.

### `RCM_Test` — test stand for remote center of motion

A simple stand, shaped like a phone's MagSafe stand, with a large hole in an angled ring (about 60° from vertical; the stand is roughly 100 mm wide and 173 mm tall). The hole stands in for the small opening the instrument has to pass through without touching the sides. It is meant for testing Remote Center of Motion (RCM) control in the future; **it has not been used yet**, since RCM control has not been built (see Next Steps).

**Suggested improvement:** add three slots on the outside of the ring as checkpoints that the instrument tip must touch before a test. Three known points define the hole's plane and center, so touching them lets the robot work out where the true center of the opening is, and RCM control can then be established about that point.

## Archived Experiments and Superseded Files

None of these are needed to run the robot. They are kept for reference.

- `applySettingTest.py`: an early experiment for configuring the prismatic motor (6) alone, using angle resolution 5 with bounded angle limits (348–3748) to cover several revolutions. Lessons kept: setting Phase bit 4 for multi-turn position feedback (used in `calibration.py` and script 07), and never writing 128 to Torque Enable to re-center, because it resets the raw position and ignores angle resolution. Approach dropped: the final scripts use angle resolution 1 with both angle limits set to 0 (true multi-turn mode). Resolution 5 is also outside the valid 1–3 range.
- `main.py`: a quick test of the new `Bus` and `Motor` classes on one motor (ID 1): sets multi-turn mode and moves one full turn and back. It uses angle resolution 4, outside the valid 1–3 range.
- `prismaticJointCalibration.py`: the first sensorless homing sequence for the prismatic carriage (motor 6 only). Replaced by `calibration.py`, which does the same homing and also range-tests motors 7–10.
- `single_motor_movement_test.py` and `single_motor_test_with_settings.py`: the first single-motor experiments, run on disk motors 10 and 9. They are where the center-offset method started; the second also tried safety and hardware angle limits. Replaced by `positionCalibration.py` and the torque characterization test.
- `positionCalibrationOOP.py` and `rangeTestOOP.py`: class-based versions of `positionCalibration.py` and `rangeTest.py`. Do not use them; `positionCalibrationOOP.py` crashes on startup.
- `template.py`: a starting skeleton for new `sms_sts` scripts.
- `quad_wheel_control.py`: an early keyboard test that spun several motors continuously.
- FeeTech's example scripts in `sms_sts/` (`ping.py`, `scan.py`, `read.py`, `write.py`, `read_write.py`, `reg_write.py`, `sync_read.py`, `sync_write.py`, `sync_read_write.py`, `wheel.py`): the vendor's SDK examples. Useful for learning the SDK; to find motors on the bus, use `scan_motors.py` instead.
- `ps4_teleop/sim.py`: the first simulation script, replaced by `simv2.py`.

## Known Issues

Every known problem in one place. Most are also described in their own file's section; the stale comments, repo hygiene and missing source files appear only here.

### Can damage settings or hardware — check before running

- **`printSettings.py` writes to EEPROM.** It passes `angleResolution=0`, and the `Motor` constructor writes any value other than 1, so running it sets Angle Resolution 0 and the multi-turn bit on motor 9. Fix: change it to 1.
- **Invalid Angle Resolution values in archived scripts.** Angle Resolution is only valid for 1–3, but `main.py` writes 4 and `applySettingTest.py` writes 5. Don't run them as-is.
- **Motor 8's offset may be stored wrong.** Register 31 stores the sign in bit 11 (range ±2047), but `positionCalibration.py` writes the negative offset (−298) as a plain integer. Read register 31 back on motor 8 to confirm its center before trusting it.
- **The torque test would move ±1400 under load.** It moves between each motor's min and max, which now come from the ±1400 range; the original test used ±800. Add a separate test-travel setting before re-running it.
- **`simHardware.py` is unstable on hardware.** Its start target is floor height (z = 0) with no speed limit; successful IK leaves the simulated robot teleported; the control loop has no sleep and floods the bus; the instrument mapping is an unverified 1:1 radian mapping that ignores cable coupling; and `wrist_yaw` is a fixed joint in the URDF. Recreate it rather than running it.

### Broken or misleading

- **`positionCalibrationOOP.py` crashes on startup:** it builds its motors before the port is open and passes `angleResolution=None`. Use `positionCalibration.py`. `rangeTestOOP.py` is also not to be used.
- **Script 07's docstring swaps the leader and follower ports** compared with `commands.md`. Check with `scan_motors.py`.
- **Running `calibration.py` before script 07 is redundant,** although `commands.md` lists it: script 07 homes the carriage itself on every start.
- **`getPS4Data` in `simv2.py` doesn't work:** both triggers send the same sign, and triggers usually rest at −1, so it can drive Z with nothing pressed. It is commented out.
- **Missing source files.** Only compiled bytecode in `__pycache__` exists for `simHardwareSync`, `simHardwareTest`, `urdf_edit` and `urdf_viewer`, and `urdf_edit` refers to a `calibrate_and_apply_home.py` that isn't in the repo. Recover these from the lab machine.

### Stale comments

- Several comments say "lead screw"; the drive is a GT2 belt.
- `prismaticJointCalibration.py` says torque stays at 12.5%; the code sets 30%.
- The torque test labels 125 as "25%"; 125 is 12.5%.
- The torque test says motor 8 goes to its minimum; the code uses its maximum.
- The URDF header says wrist roll became a fixed mount; `wrist_to_rail` is revolute.
- `calibration.py` says the offsets come from `positionCalibrationOOP.py`; they come from `positionCalibration.py`.

### Hard-coded paths and ports

- The URDF path in `simv2.py` and `simHardware.py` is hard-coded to `/home/user/Desktop/...`.
- Serial ports are hard-coded, including `DEVICENAME` in every `sms_sts` script.

### Repo hygiene

- Someone else's Windows virtual environment is committed (user `liuwei`, Python 3.11.4).
- There is a duplicate SDK zip, 805 `__pycache__` files and a `.DS_Store`, and no `.gitignore`.

### Design (for the next CAD revision)

- The carbon rod front end dovetail is too tight to remove easily.
- All four corners of the cages have poka-yoke pegs, so the top cage can still fit rotated 180°. Use three.
- Joining the top and bottom cages is fiddly, because the disk adapter sockets must line up with the instrument's disks at the same time.

## Next Steps

Work through these in order. Each depends on the one before it.

1. **Fix the simulation.** This is the biggest and first step. The simulation must not only run cleanly, but also work with PS4 teleoperation (building on `controller_input.py`).
2. **Connect the simulation to the physical robot.** Only once the simulation works cleanly and without mistakes is it safe to run PyBullet in conjunction with the real robot (what `simHardware.py` attempted).
3. **Remote Center of Motion (RCM) control.** After that, work towards RCM control, so the instrument pivots about a fixed point at the opening.

## Resources

### ⭐ [PyBullet Quickstart Guide](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit?tab=t.0#heading=h.2ye70wns7io3)

**The main reference for all the simulation work (`simv2.py`, `simHardware.py`).** The official PyBullet documentation by Erwin Coumans and Yunfei Bai. Sections most relevant to this project:

- **calculateInverseKinematics:** what `jointDamping`, `restPoses`, `jointRanges`, `maxNumIterations`, `residualThreshold` and the solver options do when tuning `moveRobot`.
- **loadURDF:** `useFixedBase` and the self-collision flags.
- **setJointMotorControl2/Array, resetJointState, getLinkState:** how joints are driven, test-placed and read back.
- **stepSimulation / setRealTimeSimulation:** how the simulation advances.
- **Transforms:** quaternion and Euler conversions and `multiplyTransforms`, used for the rotation steps.

### Reference videos

Other builds and explainers that informed this project:

- [*I Built A Surgery Robot*](https://www.youtube.com/watch?v=A_BlNA7bBxo) — Michael Reeves
- [*Building a DIY Surgical Robot*](https://www.youtube.com/watch?v=d_8rHKrwr-Q) — Bennett Stirton
- [*Building a surgery robot in my apartment*](https://www.youtube.com/watch?v=Oa4FznGK070) — ChargedHex
- [*Teardown of Surgical Robot Instrument*](https://www.youtube.com/watch?v=Xau6Qnt0wPE) — Kyle Bartholomew: how the instrument's input disks drive its joints through cables.
- [*Basics of Robotic Surgery: Da Vinci Si HD Surgical System Instructional Video*](https://www.youtube.com/watch?v=EKqYt_We4fU) — Vivek Bindal: how the real da Vinci system is used.
