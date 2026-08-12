#!/usr/bin/env python
"""Teleoperate just the SO-101 base arm (motors 1-5) from the leader arm.

The leader's get_action() reports all 6 of its motors, including
"gripper.pos" -- but the follower here (SO101Follower5DOF) only owns motors
1-5, so the gripper key is dropped before sending. Everything about motors
6-10 (custom joints, dual-gripper fan-out, keyboard control) is intentionally
out of scope for this script; it only proves out leader->follower motion for
the 5 shared arm joints.

Usage:
    06_teleoperate_base_arm.py \
        --leader-port /dev/tty.usbmodem575E0031751 --leader-id my_leader_arm \
        --follower-port /dev/tty.usbmodem585A0076841 --follower-id my_follower_arm
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from lerobot.robots.so_follower import SOFollowerRobotConfig  # noqa: E402
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig  # noqa: E402
from lerobot.utils.robot_utils import precise_sleep  # noqa: E402

from so101_follower_5dof import SO101Follower5DOF  # noqa: E402

ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--leader-port", required=True)
    parser.add_argument("--leader-id", required=True)
    parser.add_argument("--follower-port", required=True)
    parser.add_argument("--follower-id", required=True)
    parser.add_argument("--fps", type=int, default=60)
    args = parser.parse_args()

    leader = SO101Leader(SO101LeaderConfig(port=args.leader_port, id=args.leader_id))
    follower = SO101Follower5DOF(SOFollowerRobotConfig(port=args.follower_port, id=args.follower_id))

    leader.connect()
    follower.connect()

    print("Teleoperating base arm (5 DOF). Ctrl+C to stop.")
    try:
        while True:
            loop_start = time.perf_counter()

            raw_action = leader.get_action()
            arm_action = {f"{joint}.pos": raw_action[f"{joint}.pos"] for joint in ARM_JOINTS}
            follower.send_action(arm_action)

            dt = time.perf_counter() - loop_start
            precise_sleep(max(1 / args.fps - dt, 0.0))
    except KeyboardInterrupt:
        pass
    finally:
        leader.disconnect()
        follower.disconnect()


if __name__ == "__main__":
    main()
