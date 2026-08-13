Leader port: /dev/ttyACM1
Follower port: /dev/ttyACM0

scripts/01_find_ports.sh

scripts/03_calibrate_leader.sh /dev/ttyACM1 leader_arm

uv run scripts/05_calibrate_follower.py /dev/ttyACM0 follower_arm

uv run scripts/06_teleoperate_base_arm.py \
    --leader-port /dev/ttyACM1 --leader-id leader_arm \
    --follower-port /dev/ttyACM0 --follower-id follower_arm
