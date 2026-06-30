# AI Rover

A Waveshare UGV Rover (Raspberry Pi + ESP32) built on **ROS 2 Jazzy**, driven for now
with WASD-style teleop, heading toward an untethered robot whose camera recognizes a cat.

## How it works

```
  MAC (dev + keyboard)            RASPBERRY PI (ROS 2 in Docker)            ESP32 board
  ────────────────────            ──────────────────────────────            ──────────
  PyCharm / SSH  ── WiFi ──▶  ugv_driver  ──/dev/serial0 (UART JSON)──▶  motors / PID / IMU
                               ▲ sub /cmd_vel (Twist)
                               │
                          teleop_twist_keyboard  (your WASD keys, over SSH)
```

The Pi is the brain; the ESP32 actually runs the motors. The Pi controls it by writing
one tiny JSON line per command over a 115200-baud UART:

```json
{"T": 1, "L": 0.4, "R": 0.4}   // left/right wheel speed in m/s, range -0.5..+0.5
```

**`/cmd_vel` is the stable seam.** Today a human publishes Twist to it via teleop. Later,
a cat-following behavior node publishes to the *same* topic — the driver below never changes.

## Layout

```
ugv_ws/src/ugv_driver/    rclpy node: /cmd_vel -> differential-drive -> serial JSON  (permanent base)
ugv_ws/src/ugv_bringup/   launch file + driver_params.yaml
docker/                   ROS 2 Jazzy image + compose (runs on Raspberry Pi OS, no reflash)
scripts/serial_smoketest.py   ROS-free hardware check (use this first)
docs/PI_SETUP.md          one-time Pi setup — START HERE
docs/COMMANDS.md          day-to-day command cheatsheet
```

## Why Docker?

Not strictly required — but ROS 2 needs Ubuntu-like userland, and the Pi runs Raspberry Pi
OS. Running ROS 2 Jazzy in a container gets us the modern, scalable foundation **without
reflashing the Pi**, and `docker compose down` leaves the existing setup untouched.

## Getting started

1. **`docs/PI_SETUP.md`** — enable SSH + serial on the Pi, install Docker, collect a few facts.
2. From the Mac: sync this repo to the Pi, then follow **`docs/COMMANDS.md`** to build and drive.

## Roadmap

- [x] Project scaffold (driver, Docker, launch, docs)
- [ ] First light: WASD teleop moves the rover
- [ ] `ugv_perception`: camera bring-up + cat detector → publishes detections
- [ ] `ugv_behavior`: turn detections into `/cmd_vel` (follow the cat)
```
