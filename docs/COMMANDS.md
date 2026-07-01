# Command cheatsheet

## Drive the rover (the full first-light flow)

All of this runs **on the Pi** (over SSH from your Mac), from the `docker/` dir.

```bash
cd ~/rover/docker

# one-time: build image + workspace
docker compose build
docker compose run --rm rover colcon build --symlink-install

# terminal A: base driver (subscribes /cmd_vel -> serial)
docker compose up            # or `up -d` to detach

# terminal B: WASD teleop (your Mac keystrokes drive it)
# NOTE: `docker compose exec` does NOT run the image ENTRYPOINT, so ROS isn't sourced
# and `ros2` won't be found. Invoke /entrypoint.sh explicitly (it sources ROS, then execs).
docker compose exec rover /entrypoint.sh \
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p repeat_rate:=15.0
```

> `repeat_rate:=15.0` makes teleop re-publish the current command ~15x/s. Without
> it teleop only publishes on each keypress and the safety watchdog (0.5 s) would
> stop the rover between presses. With it, holding a direction keeps moving;
> press `k` / space to stop.

teleop_twist_keyboard keys: `i`=forward, `,`=back, `j`/`l`=turn, `k`/space=stop,
`q`/`z`=raise/lower max speeds. (A literal W/A/S/D node is a later nicety — it
publishes to the same `/cmd_vel`, so nothing else changes.)

## Inspect / debug the ROS graph

```bash
# (exec bypasses the entrypoint, so prefix each with /entrypoint.sh to source ROS)
docker compose exec rover /entrypoint.sh ros2 node list
docker compose exec rover /entrypoint.sh ros2 topic echo /cmd_vel
docker compose exec rover /entrypoint.sh ros2 param dump /ugv_driver
```

## Hardware-only check (no ROS), straight to the ESP32

Run this on the **Pi host** (simplest — no container, no ROS in the way). It
streams the command and prints the ESP32's battery voltage + measured wheel
speeds, so you get a real link/power/motor check, not just "no error".

**UART gotcha (Pi 5):** the ESP32 is on `/dev/ttyAMA0` (GPIO14/15 PL011). Do NOT
use `/dev/serial0` on the host — it is aliased to `/dev/ttyAMA10`, a different,
unconnected UART, so commands to it silently do nothing. (Inside the container
`/dev/serial0` is correct — compose maps host `ttyAMA0` → `/dev/serial0`.)

**Free the port first:** the stock Waveshare app is started from a user `crontab`
`@reboot` entry (not a systemd service), so it comes back on every boot. Stop it
for this session by killing its process (do NOT edit anything under `~/ugv_rpi`):

```bash
# find + stop the stock web app (also frees :5000)
PID=$(pgrep -f 'ugv_rpi/[a]pp\.py'); echo "app.py = ${PID:-none}"; [ -n "$PID" ] && kill "$PID"

# python3-serial ships on the Pi; then run the streaming smoketest on the RIGHT port
python3 ~/rover/scripts/serial_smoketest.py --port /dev/ttyAMA0 --left 0.2 --right 0.2 --duration 1.5
```

## Tear down (leaves Raspberry Pi OS untouched)

```bash
docker compose down
```

## Rebuild after editing the driver

Python nodes built with `--symlink-install` pick up edits to existing files on
restart — just `docker compose restart`. Re-run `colcon build` only after adding
files, changing `setup.py`/`package.xml`, or editing launch/params installs.

## Sync Mac → Pi (the Mac is the source of truth)

```bash
rsync -a --delete \
  --exclude '.git' --exclude '.idea' --exclude '.venv' \
  --exclude 'ugv_ws/build' --exclude 'ugv_ws/install' --exclude 'ugv_ws/log' \
  --exclude '__pycache__' --exclude '*.pyc' --exclude '.DS_Store' \
  "<this repo>/" rover:rover/
```

## Local IDE setup on the Mac (autocomplete only — the rover runs in Docker)

PyCharm can't index a remote Docker interpreter over an SSH daemon (JetBrains bug
PY-52770 / PY-60495 — its skeleton generator fails across the ephemeral helper
containers). Instead, mirror the container's Python API into a local venv that the
editor can index reliably:

```bash
brew install python@3.12          # one-time: the Mac ships only 3.9
bash scripts/setup_dev_venv.sh    # builds .venv + mirrors ROS 2 packages from the image
```

Then set PyCharm's **Project Interpreter** to `./.venv/bin/python`. This venv is for
autocomplete/inspections only — you never *run* with it; the rover always runs in Docker
on the Pi. Re-run the script after rebuilding/upgrading the image to refresh the mirror.
