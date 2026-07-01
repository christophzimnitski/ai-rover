#!/usr/bin/env bash
# Follow the rover container logs (Ctrl-C to stop). Handy for watching joy_supervisor
# restart the pad, driver bring-up, or telemetry warnings.
#
#   scripts/logs.sh
set -euo pipefail

REMOTE="${ROVER_SSH:-rover}"
ssh "$REMOTE" 'cd ~/rover/docker && docker compose logs -f --tail=80 rover'
