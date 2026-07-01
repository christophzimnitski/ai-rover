#!/usr/bin/env bash
# Manual controller reconnect / big-hammer recovery.
#
# The joy_supervisor node now auto-recovers the pad on sleep/wake and dongle
# re-enumeration, so you rarely need this. Keep it as the fallback if the whole
# container wedges. Wake the pad first, then run this.
#
#   scripts/reconnect.sh
set -euo pipefail

REMOTE="${ROVER_SSH:-rover}"
echo "restarting rover container (wake the pad first) ..."
ssh "$REMOTE" 'cd ~/rover/docker && docker compose restart rover'
