# SO-101 base arm (LeRobot)

Isolated LeRobot install for bringing up just the SO-101 leader/follower base
arm (motors 1-5 on each side). Completely separate from `../STServo_Python`
— nothing here touches that project's venv, code, or the follower's motors
6-10 (custom joints owned by that other code).

## Why a 5-motor follower

This follower's motor 6 is not a gripper — it's a different custom joint —
and motors 7-10 belong to other mechanisms (dual gripper fan-out from the
leader's motor 6, plus keyboard-controlled joints) that aren't wired up yet.
The stock `lerobot.robots.so_follower.SOFollower` hardcodes a 6-motor bus
including a "gripper" on id 6, so it can't be used directly on the follower
side without colliding with that. `so101_follower_5dof.py` defines
`SO101Follower5DOF`, which reuses everything from the stock class except the
bus construction, trimmed to just ids 1-5. The leader is untouched stock
(`lerobot.teleoperators.so_leader.SO101Leader`) since it really is a plain
6-motor SO-101 leader.

Calibration files for the trimmed follower are stored under
`~/.cache/huggingface/lerobot/calibration/robots/so101_follower_5dof/<id>.json`
— a different robot name than stock `so101_follower`, so it can never
collide with a 6-motor calibration file.

## Setup (one-time)

```bash
uv pip install --python "$PWD/.venv/bin/python3" 'lerobot[feetech,core_scripts]'
```

Already done — this venv has it installed.

## Bring-up order

Run scripts from this directory (`so101_lerobot/`) so relative paths resolve.

1. **Find each arm's USB port** — run once, follow the prompts to unplug/replug:
   ```bash
   scripts/01_find_ports.sh
   ```

2. **Leader: assign motor IDs 1-6** (stock SO-101 leader, all 6 motors).
   Connect the controller board to one motor at a time as prompted, starting
   from the gripper:
   ```bash
   scripts/02_setup_motors_leader.sh <leader-port> <leader-arm-id>
   ```

3. **Leader: calibrate** — move to the middle of its range, press enter, then
   move each joint through its full range of motion:
   ```bash
   scripts/03_calibrate_leader.sh <leader-port> <leader-arm-id>
   ```

4. **Follower: assign motor IDs 1-5 only.** Only connect the controller board
   to the 5 base-arm motors as prompted (shoulder_pan..wrist_roll) — do not
   run this with motors 6-10 in a state where this could reassign their IDs;
   this script only ever addresses ids 1-5:
   ```bash
   scripts/04_setup_motors_follower.py <follower-port> <follower-arm-id>
   ```

5. **Follower: calibrate motors 1-5 only**:
   ```bash
   scripts/05_calibrate_follower.py <follower-port> <follower-arm-id>
   ```

6. **Teleoperate the base arm** (leader's 5 arm joints -> follower's 5 arm
   joints; leader's gripper reading is read but intentionally dropped, not
   sent anywhere yet):
   ```bash
   scripts/06_teleoperate_base_arm.py \
       --leader-port <leader-port> --leader-id <leader-arm-id> \
       --follower-port <follower-port> --follower-id <follower-arm-id>
   ```
   Ctrl+C to stop.

## Not done here (future work)

- Fanning the leader's gripper (motor 6) out to follower motors 9 and 10.
- Keyboard control of follower motors 6/7/8.
- Remote-center-of-motion kinematics.
- Dataset recording / imitation-learning policy training via
  `lerobot-record` / `lerobot-train`, once teleoperation of the full system
  works end-to-end.
