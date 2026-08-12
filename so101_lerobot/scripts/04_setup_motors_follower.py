#!/usr/bin/env python
"""Assign IDs 1-5 to the follower arm's base-arm motors only.

Equivalent to `lerobot-setup-motors --robot.type=so101_follower`, except it
never touches a "gripper" motor on id 6 -- on this robot, follower id 6 is a
different custom joint owned by other code, and this script must not
interfere with it. Only run this with the controller board connected to
motors 1-5 (shoulder_pan..wrist_roll); leave 6-10 alone.

Usage: 04_setup_motors_follower.py <port> <arm-id>
Example: 04_setup_motors_follower.py /dev/tty.usbmodem585A0076841 my_follower_arm
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from lerobot.robots.so_follower import SOFollowerRobotConfig  # noqa: E402

from so101_follower_5dof import SO101Follower5DOF  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="Serial port for the follower arm, e.g. /dev/tty.usbmodem585A0076841")
    parser.add_argument("arm_id", help="Unique name for this follower arm")
    args = parser.parse_args()

    config = SOFollowerRobotConfig(port=args.port, id=args.arm_id)
    follower = SO101Follower5DOF(config)
    follower.setup_motors()


if __name__ == "__main__":
    main()
