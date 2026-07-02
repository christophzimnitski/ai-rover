#!/usr/bin/env bash
# First-run bootstrap for a FRESH Pi or NEW storage (SD card or SSD): sync, build the
# Docker image, build the colcon workspace, and bring the stack up. Run this once after
# provisioning; afterwards the container autostarts on boot and the everyday loop is
# reload.sh (edits) / build.sh (new packages) / image.sh (Docker changes).
#
# Why this exists: install/ (the built workspace) is gitignored and lives only on the
# Pi's storage. Move to a new SD/SSD and it's gone -- the container would come up and
# crash-loop with "package 'ugv_bringup' not found". This rebuilds it from nothing.
#
#   scripts/bootstrap.sh
set -euo pipefail

REMOTE="${ROVER_SSH:-rover}"
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

"$DIR/sync.sh"
echo "installing camera udev rule (no USB autosuspend, prevents the C950 wedging) ..."
ssh "$REMOTE" 'sudo cp ~/rover/scripts/99-rover-camera-no-autosuspend.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger'
echo "building rover image ..."
ssh "$REMOTE" 'cd ~/rover/docker && docker compose build'
echo "building colcon workspace (symlink-install) ..."
ssh "$REMOTE" 'cd ~/rover/docker && docker compose run --rm rover colcon build --symlink-install'
echo "bringing the stack up ..."
ssh "$REMOTE" 'cd ~/rover/docker && docker compose up -d'
echo "bootstrap complete: image + workspace built, stack up. Check it: scripts/verify.sh"
