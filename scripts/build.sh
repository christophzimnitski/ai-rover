#!/usr/bin/env bash
# Sync Mac->Pi, rebuild the colcon workspace in the container, then restart.
# Needed when you add a NEW node/entry point or edit setup.py / package.xml /
# add a launch or config file. For plain tuning/code edits, scripts/reload.sh
# (no rebuild) is enough and faster.
#
#   scripts/build.sh
set -euo pipefail

REMOTE="${ROVER_SSH:-rover}"
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

"$DIR/sync.sh"
echo "colcon build (symlink-install) ..."
ssh "$REMOTE" 'cd ~/rover/docker && docker compose run --rm rover colcon build --symlink-install'
echo "restarting rover container ..."
ssh "$REMOTE" 'cd ~/rover/docker && docker compose restart rover'
echo "done. tail logs with: scripts/logs.sh"
