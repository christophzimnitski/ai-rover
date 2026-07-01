#!/usr/bin/env bash
# Push the Mac working tree -> Pi (~/rover). The Mac is the source of truth: we edit
# here, rsync there, and run in Docker on the Pi (no ROS on macOS).
#
#   scripts/sync.sh
#
# --delete keeps the Pi a mirror of the Mac, EXCEPT we exclude top-level ugv_ws/*.py
# so the throwaway diagnostic scripts that live only on the Pi (js_raw.py, jsdrift.py,
# cmd_mon.py, ...) survive a sync. Real code lives under ugv_ws/src and is always synced.
#
# Override the SSH host with ROVER_SSH=<host> if your ~/.ssh/config alias isn't "rover".
set -euo pipefail

REMOTE="${ROVER_SSH:-rover}"
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

rsync -a --delete \
  --exclude '.git' \
  --exclude '.idea' \
  --exclude '.venv' \
  --exclude 'ugv_ws/build' \
  --exclude 'ugv_ws/install' \
  --exclude 'ugv_ws/log' \
  --exclude '__pycache__' \
  --exclude '*.pyc' \
  --exclude '.DS_Store' \
  --exclude 'ugv_ws/*.py' \
  "$PROJECT_DIR/" "$REMOTE:rover/"

echo "synced -> $REMOTE:rover/"
