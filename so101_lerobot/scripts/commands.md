Leader port: /dev/ttyACM1
Follower port: /dev/ttyACM0

scripts/01_find_ports.sh

scripts/03_calibrate_leader.sh /dev/ttyACM1 leader_arm

uv run scripts/05_calibrate_follower.py /dev/ttyACM0 follower_arm

cd ../STServo_Python/stservo-env/sms_sts
uv run calibration.py

IMPORTANT: do NOT unplug or power-cycle the follower board after this step. Torque
enable/limit on motors 6-10 live in volatile SRAM on the servos themselves - they hold as
long as power stays on, which is what lets teleop below leave those motors alone safely.
If power is lost, re-run calibration.py before teleoperating again.


cd ../../../so101_lerobot
uv run scripts/06_teleoperate_base_arm.py \
    --leader-port /dev/ttyACM1 --leader-id leader_arm \
    --follower-port /dev/ttyACM0 --follower-id follower_arm
