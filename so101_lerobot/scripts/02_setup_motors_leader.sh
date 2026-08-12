#!/usr/bin/env bash
# Assign IDs 1-6 to the leader arm's motors (all 6, stock SO-101 leader layout).
# Usage: 02_setup_motors_leader.sh <port> <arm-id>
# Example: 02_setup_motors_leader.sh /dev/tty.usbmodem575E0031751 my_leader_arm
set -euo pipefail
cd "$(dirname "$0")/.."

PORT="${1:?Usage: $0 <port> <arm-id>}"
ARM_ID="${2:?Usage: $0 <port> <arm-id>}"

.venv/bin/lerobot-setup-motors \
    --teleop.type=so101_leader \
    --teleop.port="$PORT" \
    --teleop.id="$ARM_ID"
