#!/usr/bin/env bash
# Identify which USB serial port belongs to which arm.
# Run this once per arm (leader, then follower) - it walks you through
# unplugging one at a time so it can tell them apart.
#
# Copied from so101_lerobot/scripts/01_find_ports.sh -- adjusted to use THIS
# folder's own .venv (ps4_teleop/.venv) since this copy lives flat in
# ps4_teleop/, not nested one level under a project root.
set -euo pipefail
cd "$(dirname "$0")"
.venv/bin/lerobot-find-port
