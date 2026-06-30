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
docker compose exec rover \
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
docker compose exec rover ros2 node list
docker compose exec rover ros2 topic echo /cmd_vel
docker compose exec rover ros2 param dump /ugv_driver
```

## Hardware-only check (no ROS), straight to the ESP32

Run this on the **Pi host** (simplest — no container, no ROS in the way). Make
sure nothing else holds the port first (e.g. stop Waveshare's stock service):

```bash
sudo apt install -y python3-serial
python3 ~/rover/scripts/serial_smoketest.py --port /dev/serial0 --left 0.2 --right 0.2 --duration 1.5
```

## Tear down (leaves Raspberry Pi OS untouched)

```bash
docker compose down
```

## Rebuild after editing the driver

Python nodes built with `--symlink-install` pick up edits to existing files on
restart — just `docker compose restart`. Re-run `colcon build` only after adding
files, changing `setup.py`/`package.xml`, or editing launch/params installs.
