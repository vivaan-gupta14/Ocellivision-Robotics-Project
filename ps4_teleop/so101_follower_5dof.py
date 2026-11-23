"""Custom SO-101 follower that only owns motor IDs 1-5 (the base arm joints).

Copied from so101_lerobot/so101_follower_5dof.py -- required by 05_calibrate_follower.py's
copy in this folder. Kept byte-for-byte identical; only ever used for calibration here, not
imported by simHardware.py (which copies its own constants/logic instead per instruction).

The stock `lerobot.robots.so_follower.SOFollower` hardcodes a 6-motor bus
(shoulder_pan..wrist_roll, ids 1-5, plus a "gripper" motor on id 6). On this
robot, follower id 6 is a different, custom joint (not a gripper) and ids
7-10 are further custom motors owned by other code entirely. Touching any of
them here would fight that other setup, so this class re-implements just the
bus construction with the gripper entry dropped, and reuses everything else
from SOFollower unchanged (calibrate/configure/setup_motors/get_observation/
send_action all just iterate self.bus.motors, so they work as-is on 5 motors).

Calibration files for this class are stored under a distinct robot name
("so101_follower_5dof") so they can never collide with a stock 6-motor
so101_follower calibration file.
"""

from lerobot.cameras import make_cameras_from_configs
from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.robots.robot import Robot
from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig


class SO101Follower5DOF(SOFollower):
    """SO-101 follower limited to the 5 base-arm joints (motor ids 1-5).

    Config is the plain SOFollowerRobotConfig (port, id, ...) -- there's no
    separate config type since this class is only ever instantiated directly
    in these scripts, never through lerobot's --robot.type= CLI registry.
    """

    config_class = SOFollowerRobotConfig
    name = "so101_follower_5dof"

    def __init__(self, config: SOFollowerRobotConfig):
        # Deliberately skip SOFollower.__init__ (it hardcodes a 6-motor bus
        # including "gripper" on id 6) and re-run what it does, minus that
        # motor.
        Robot.__init__(self, config)
        self.config = config
        norm_mode_body = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100
        self.bus = FeetechMotorsBus(
            port=self.config.port,
            motors={
                "shoulder_pan": Motor(1, "sts3215", norm_mode_body),
                "shoulder_lift": Motor(2, "sts3215", norm_mode_body),
                "elbow_flex": Motor(3, "sts3215", norm_mode_body),
                "wrist_flex": Motor(4, "sts3215", norm_mode_body),
                "wrist_roll": Motor(5, "sts3215", norm_mode_body),
            },
            calibration=self.calibration,
        )
        self.cameras = make_cameras_from_configs(config.cameras)
