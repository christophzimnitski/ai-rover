#!/usr/bin/env bash
# Sync Mac->Pi, REBUILD the Docker image, then bring the stack back up. This is the
# loop for changes to docker/Dockerfile or docker/docker-compose.yml (new apt deps,
# base-image pin, new mounts/services) -- reload.sh/build.sh do NOT rebuild the image.
#
#   scripts/image.sh
#
# After an image rebuild that also adds/changes ROS packages (e.g. a new msg package),
# follow with scripts/build.sh to colcon-build the workspace against the new image.
set -euo pipefail

REMOTE="${ROVER_SSH:-rover}"
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

"$DIR/sync.sh"
echo "rebuilding rover image (docker compose build) ..."
ssh "$REMOTE" 'cd ~/rover/docker && docker compose build'
echo "bringing the stack up (docker compose up -d) ..."
ssh "$REMOTE" 'cd ~/rover/docker && docker compose up -d'
echo "done. If you added/changed ROS packages, now run: scripts/build.sh"
