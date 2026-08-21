#!/usr/bin/env python
"""Calibrate the follower arm's 5 base-arm motors only (ids 1-5).

Equivalent to `lerobot-calibrate --robot.type=so101_follower`, restricted to
the 5-motor SO101Follower5DOF class so it never reads/writes/homes motors
6-10, which belong to other custom joints on this follower and already have
their own calibration flow.

Copied from so101_lerobot/scripts/05_calibrate_follower.py -- the sys.path
insertion for importing so101_follower_5dof is dropped since that module now
lives alongside this script in this same folder (flat layout, no scripts/
subfolder), so a plain same-directory import already finds it.

Usage: 05_calibrate_follower.py <port> <arm-id>
The <arm-id> must match what you used in so101_lerobot's 04_setup_motors_follower.py.
"""

import argparse

from lerobot.robots.so_follower import SOFollowerRobotConfig

from so101_follower_5dof import SO101Follower5DOF


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="Serial port for the follower arm, e.g. /dev/tty.usbmodem585A0076841")
    parser.add_argument("arm_id", help="Unique name for this follower arm (must match setup step)")
    args = parser.parse_args()

    config = SOFollowerRobotConfig(port=args.port, id=args.arm_id)
    follower = SO101Follower5DOF(config)
    follower.connect(calibrate=False)
    try:
        follower.calibrate()
    finally:
        follower.disconnect()


if __name__ == "__main__":
    main()
