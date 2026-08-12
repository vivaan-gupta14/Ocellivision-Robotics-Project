#!/usr/bin/env bash
# Identify which USB serial port belongs to which arm.
# Run this once per arm (leader, then follower) - it walks you through
# unplugging one at a time so it can tell them apart.
set -euo pipefail
cd "$(dirname "$0")/.."
.venv/bin/lerobot-find-port
