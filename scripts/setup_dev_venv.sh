#!/usr/bin/env bash
#
# Local IDE virtualenv for the Mac -- code-intelligence ONLY (autocomplete / inspections).
#
# Why this exists: PyCharm cannot index a remote Docker interpreter over an SSH-remote daemon
# (JetBrains bugs PY-52770 / PY-60495 -- its skeleton generator can't persist state across the
# ephemeral helper containers). So instead of indexing across the network, we give the editor a
# LOCAL mirror of the container's Python API: a venv holding pyserial plus a source-only copy of
# the ROS 2 Python packages pulled from the image.
#
# This venv is NEVER used to run anything. The rover always runs inside Docker on the Pi. The
# venv exists purely so the editor can resolve rclpy / message packages / pyserial. (The ROS C
# extensions copied in are aarch64-linux ELF and are inert on macOS -- PyCharm only reads the
# .py source for completion, so that's fine.)
#
# Prereqs:  python@3.12 on PATH (brew install python@3.12)  and  `ssh rover` working.
# Usage:    bash scripts/setup_dev_venv.sh
# Refresh:  re-run after rebuilding or upgrading the image.
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
VENV="$PROJECT_DIR/.venv"
SSH_HOST="${ROVER_SSH:-rover}"
IMAGE="${ROVER_IMAGE:-ugv-rover:jazzy}"
ROS_SP="/opt/ros/jazzy/lib/python3.12/site-packages"

PY="$(command -v python3.12 || true)"
[ -n "$PY" ] || { echo "ERROR: python3.12 not found on PATH. Run: brew install python@3.12"; exit 1; }

echo "==> (Re)creating venv at: $VENV"
rm -rf "$VENV"
"$PY" -m venv "$VENV"

echo "==> Installing pyserial (the driver's only non-ROS dependency)"
"$VENV/bin/pip" install --quiet pyserial

VENV_SP="$("$VENV/bin/python" -c 'import site; print(site.getsitepackages()[0])')"

echo "==> Mirroring ROS 2 Python packages from $IMAGE:$ROS_SP"
echo "    -> $VENV_SP  (source-only, for autocomplete)"
ssh "$SSH_HOST" "docker run --rm --entrypoint tar '$IMAGE' -C '$(dirname "$ROS_SP")' -cf - '$(basename "$ROS_SP")'" \
  | tar -C "$VENV_SP" -xf - --strip-components=1

echo "==> Verifying (checking sources are present; not importing -- the C exts are Linux-only):"
ok=1
for d in rclpy geometry_msgs std_msgs builtin_interfaces; do
  if [ -f "$VENV_SP/$d/__init__.py" ]; then echo "    $d: ok"; else echo "    $d: MISSING"; ok=0; fi
done
"$VENV/bin/python" -c "import serial; print('    serial (pyserial): ok')" || ok=0

echo
if [ "$ok" -eq 1 ]; then
  echo "Done. In PyCharm, set the Project Interpreter to:"
  echo "    $VENV/bin/python"
  echo "(Remove the broken Docker/Compose interpreter. The rover still runs in Docker on the Pi.)"
else
  echo "Something is missing above -- check that 'ssh $SSH_HOST' works and the image is built."
  exit 1
fi
