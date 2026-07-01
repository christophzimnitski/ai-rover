#!/usr/bin/env bash
# Apply a change: sync Mac->Pi, then restart the rover container so ROS re-reads it.
# This is the everyday loop for TUNING edits (config/teleop_joy.yaml,
# config/driver_params.yaml) and plain code edits -- symlink-install means no rebuild.
#
#   scripts/reload.sh
#
# Use scripts/build.sh instead when you add a NEW node/entry point or edit
# setup.py / package.xml / a new launch file (those need a colcon build).
set -euo pipefail

REMOTE="${ROVER_SSH:-rover}"
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

"$DIR/sync.sh"
echo "restarting rover container ..."
ssh "$REMOTE" 'cd ~/rover/docker && docker compose restart rover'
echo "done. tail logs with: scripts/logs.sh"
