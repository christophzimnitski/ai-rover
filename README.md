# AI Rover

A Waveshare UGV Rover (Raspberry Pi + ESP32) built on **ROS 2 Jazzy**, driven with a
2.4 GHz gamepad, heading toward an untethered robot whose camera recognizes a cat.

## How it works

```
  MAC (dev)                     RASPBERRY PI (ROS 2 in Docker)              ESP32 board
  ─────────                     ──────────────────────────────             ──────────
  PyCharm / SSH ── WiFi ──▶  joy_supervisor → joy_node                 motors / PID / IMU
                             → teleop_twist_joy ─┐                            ▲  │
                                                 ▼                            │  │ UART
                             ugv_driver ── /dev/serial0 (JSON) ──────────────┘  │ (JSON)
                               ▲ sub /cmd_vel (Twist)                           │
                               └──────── /rover/telemetry + /rover/battery ◀────┘
```

A GameSir gamepad (2.4 GHz USB dongle → `/dev/input/js0`) feeds `joy_node`;
`teleop_twist_joy` turns sticks into `geometry_msgs/Twist` on `/cmd_vel`. Left stick =
speed, right stick = direction, **hold LT to move** (deadman), RT = turbo. `joy_supervisor`
keeps `joy_node` alive across pad sleep/wake and dongle re-enumeration.

The Pi is the brain; the ESP32 runs the motors. `ugv_driver` writes one tiny JSON line
per command over a 115200-baud UART, and reads the board's feedback back:

```json
{"T": 1, "L": 0.4, "R": 0.4}   // left/right wheel command, range -0.5..+0.5
```

The stock firmware maps that command **open-loop to PWM** (see `docs/FIRMWARE.md`), so
the driver closes the speed loop itself: a slow per-wheel trim (`wheel_trim`) compares
the board's encoder feedback with the command and nudges the output until commanded
m/s is physically true — on tile, carpet, anywhere (this also removes the old
stick-then-lurch behavior on high-friction surfaces).

**`/cmd_vel` is the stable seam.** Today the gamepad publishes Twist to it via teleop.
Later, a cat-following behavior node publishes to the *same* topic — the driver below
never changes. The dashboard's on-screen joystick publishes `/web_cmd_vel`, a secondary
input the driver only accepts while `/cmd_vel` is quiet (the gamepad always wins).
The board's feedback (wheel speeds, attitude, voltage) is republished as
`/rover/telemetry` (typed `ugv_msgs/Telemetry`) + `/rover/battery`, link health as
`/rover/link`. The driver also exposes the board's accessories — `/rover/lights`
(headlights), `/rover/oled` (status display), `/rover/estop` (software e-stop latch) —
and arms the firmware's own heartbeat so the board halts itself if the Pi stack dies.

## Layout

```
ugv_ws/src/ugv_driver/    rclpy node: /cmd_vel -> diff-drive -> serial JSON, + telemetry read-back
    kinematics.py         ROS-free Twist->wheels math (unit-tested)
    serial_link.py        ROS-free ESP32 serial wrapper (command + telemetry)
ugv_ws/src/ugv_bringup/   launch files, driver/teleop params, joy_supervisor
docker/                   ROS 2 Jazzy image + compose (runs on Raspberry Pi OS, no reflash)
scripts/                  dev loop: sync / reload / build / verify / logs (+ serial_smoketest, diag)
tests/                    off-site pytest suite for the ROS-free logic (never runs on the Pi)
webapp/                   zero-dependency dashboard (video, telemetry, drive pad, lights, e-stop)
firmware/                 our 3-line patch for the stock ESP32 firmware (see docs/REFLASH.md)
docs/PI_SETUP.md          one-time Pi setup — START HERE
docs/COMMANDS.md          day-to-day command cheatsheet
docs/FIRMWARE.md          what the ESP32 firmware really does (open-loop PWM, IMU state, T-codes)
docs/REFLASH.md           ESP32 reflash procedure + post-flash safety checklist
```

## Why Docker?

ROS 2 needs an Ubuntu-like userland and the Pi runs Raspberry Pi OS. Running ROS 2 Jazzy
in a container gets us the modern, scalable foundation **without reflashing the Pi**, and
`docker compose down` leaves the existing setup untouched.

## Dev loop (Mac is the source of truth)

Edit on the Mac → `rsync` to the Pi → run in Docker on the Pi (no ROS on macOS).

- Edited a `.py` or a config `.yaml` → `scripts/reload.sh` (sync + restart)
- Added a node / message / edited `setup.py`/`package.xml`/launch → `scripts/build.sh`
- Changed the Dockerfile/compose → `scripts/image.sh` (rebuild), then `scripts/build.sh`
- Fresh Pi / new SD or SSD (first run) → `scripts/bootstrap.sh` (image + workspace + up)
- Health check / logs → `scripts/verify.sh` · `scripts/logs.sh`

The `tests/` suite is a separate off-site check (see `.github/workflows/tests.yml`); it runs
on the Mac / in CI and the rover never depends on it.

## Getting started

1. **`docs/PI_SETUP.md`** — enable SSH + serial on the Pi, install Docker, collect a few facts.
2. From the Mac: sync this repo to the Pi, then follow **`docs/COMMANDS.md`** to build and drive.

## Roadmap

- [x] Project scaffold (driver, Docker, launch, docs)
- [x] First light: teleop moves the rover
- [x] Gamepad teleop (analog speed + steering, deadman/turbo, auto-recovery)
- [x] ESP32 telemetry read-back (typed `/rover/telemetry` + `/rover/battery`)
- [x] Web dashboard: camera + live telemetry + drive pad / lights / OLED / e-stop
- [x] Pi-side speed loop (`wheel_trim`) over the open-loop firmware
- [ ] Reflash `ugv_base_general` → trustworthy IMU (roll/pitch/yaw), then `IMU_TRUSTED=true`
- [ ] `ugv_perception`: cat detector → publishes detections (boxes onto the dashboard overlay)
- [ ] `ugv_behavior`: turn detections into `/cmd_vel` (follow the cat)
