#!/usr/bin/env bash
# Quick health check on the Pi: is the pad seen, is /joy flowing, is ESP32
# telemetry arriving? Safe to run any time (read-only).
#
#   scripts/verify.sh
set -euo pipefail

REMOTE="${ROVER_SSH:-rover}"
ssh "$REMOTE" 'cd ~/rover/docker && \
  echo "--- js input devices ---"; \
  ls /dev/input/js* 2>/dev/null || echo "(no js device: pad asleep/disconnected)"; \
  echo "--- /joy (gamepad) ---"; \
  timeout 5 docker compose exec -T rover /entrypoint.sh bash -c "ros2 topic echo /joy --once" 2>/dev/null \
    || echo "(no /joy within 5s)"; \
  echo "--- /battery (ESP32) ---"; \
  timeout 5 docker compose exec -T rover /entrypoint.sh bash -c "ros2 topic echo /battery --once" 2>/dev/null \
    || echo "(no /battery within 5s)"; \
  echo "--- /rover/telemetry (ESP32 raw JSON) ---"; \
  timeout 5 docker compose exec -T rover /entrypoint.sh bash -c "ros2 topic echo /rover/telemetry --once" 2>/dev/null \
    || echo "(no telemetry within 5s)"'
