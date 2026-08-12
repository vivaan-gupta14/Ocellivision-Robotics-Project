#!/usr/bin/env bash
# Calibrate all 6 leader motors (stock lerobot flow: middle position, then
# each joint through its full range of motion).
# Usage: 03_calibrate_leader.sh <port> <arm-id>
# The <arm-id> must match what you used in 02_setup_motors_leader.sh.
set -euo pipefail
cd "$(dirname "$0")/.."

PORT="${1:?Usage: $0 <port> <arm-id>}"
ARM_ID="${2:?Usage: $0 <port> <arm-id>}"

.venv/bin/lerobot-calibrate \
    --teleop.type=so101_leader \
    --teleop.port="$PORT" \
    --teleop.id="$ARM_ID"
