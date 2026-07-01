# Rover roadmap

Living plan for the Waveshare UGV Rover. End goal: an untethered rover whose
camera recognizes the cat. Architecture principle: **everything talks to
`/cmd_vel`** (`geometry_msgs/Twist`); the `ugv_driver` node is the only thing that
speaks the ESP32's serial JSON, and it never changes as we add capability on top.

## Where we are (2026-07-01)

- **Stages 0–4 done. First-light achieved.** `/cmd_vel → ugv_driver → serial →
  ESP32 → motors` verified, including keyboard teleop.
- Hardware healthy: battery **12.25 V**, bidirectional serial link, encoder
  feedback (`{"T":1001,...,"L","R","v"}`) confirmed.
- Correct UART on this Pi 5 is **`/dev/ttyAMA0`** (compose maps it to the
  container's `/dev/serial0`). See `docs/PI_SETUP.md` for the gotcha.

## Issues found at first-light (marked)

| # | Issue | Status |
|---|-------|--------|
| A | Won't turn at low speed (static friction needs more current) | **Fixed** — `min_wheel_speed` in the driver bumps any non-zero wheel command up to a floor (default 0.10 m/s). Tune the value on the floor. |
| B | ~1 s stop lag with keyboard teleop | **Resolved by design** — root cause is that terminals have no key-up event over SSH, so teleop can't detect release. The Bluetooth gamepad (next milestone) publishes continuously and zeroes instantly on stick-release. Not worth band-aiding the keyboard. |
| C | "Holonomic mode" confusing | **N/A** — that mode is for mecanum/omni robots that strafe sideways (`linear.y`). Ours is differential drive and the driver ignores `linear.y` by design. Ignore holonomic mode. |

## Next steps (ordered)

### 1. Gamepad teleop — the chosen input (NEXT)
Replace keyboard teleop with a generic Bluetooth controller. Fixes issue B and
gives analog speed control.
- Pair the controller on the Pi host: `bluetoothctl` → `scan on` → `pair`/`trust`/
  `connect`; confirm it shows up as `/dev/input/js0` (`ls /dev/input/`, test with
  `jstest`).
- Add ROS packages to the image: `ros-jazzy-joy` + `ros-jazzy-teleop-twist-joy`.
- Give the container access to the pad: pass `/dev/input` (add to compose
  `devices:` or bind-mount `/dev/input`).
- `joy_node` → `teleop_twist_joy` → `/cmd_vel`. Add a `joystick.launch.py` +
  a config mapping: left stick Y = `linear.x`, right stick X = `angular.z`,
  a deadman/turbo button, and a stick deadzone (so centred = exact zero).
- Map it into `rover.launch.py` (or a second launch file) so `docker compose up`
  brings up driver + joy together.

### 2. Tune on the floor
- `min_wheel_speed`: lower it toward the point where wheels *just* reliably start.
- `angular_gain`: raise for snappier turns if needed.
- Confirm the 0.175 m `track_width` gives roughly correct turn rates.

### 3. Odometry + robot model (foundation for autonomy)
- Publish wheel **odometry** from the ESP32 encoder feedback (`{"T":1001}` L/R) →
  `/odom` + `tf` (odom→base_link). This is what navigation/SLAM needs.
- Minimal **URDF** (base + wheels) for `robot_state_publisher` and rviz.

### 4. Camera bring-up
- Add the Pi camera (or USB cam) as a ROS node publishing `sensor_msgs/Image`
  (`image_transport`), viewable in rviz / rqt.

### 5. Cat perception
- On-device detection (lightweight model, e.g. a small YOLO/MobileNet) node that
  consumes the image topic and publishes cat detections (bounding box / bearing).
- Mind the Pi 5's compute/thermal limits (heatsink+fan; consider an accelerator).

### 6. Follow behavior
- Behavior node: detection → a `/cmd_vel` command that turns toward and approaches
  the cat. Because it publishes `/cmd_vel`, the driver is untouched.

### 7. Untethered autonomy hardening
- Battery monitoring/telemetry (voltage already available), low-battery safe-stop.
- Remote access for a roaming robot: Tailscale + SSH hardening (deferred earlier).
- Autostart our stack on boot (our replacement for the stock crontab entry).

## Dev-environment housekeeping (independent of the above)

- Commit the dev-env changes (`scripts/setup_dev_venv.sh`, docs) + this session's
  fixes (UART, driver, roadmap) — run the repo commit gate first.
- PyCharm **Deployment** auto-sync Mac→Pi on save (replaces manual rsync). GUI work.
- Optional one-click "run rover in Docker" PyCharm run configuration.

## Reference: the stock Waveshare app (`~/ugv_rpi`, read-only)
Keep it as documentation only — do **not** edit/delete it. It holds the confirmed
ESP32 `T`-code protocol and a capabilities notebook. It autostarts from the user
crontab (`@reboot app.py` + `start_jupyter.sh`); stop it with `kill <pid>` (find via
`pgrep -f 'ugv_rpi/[a]pp.py'`) when it would contend for hardware. It is a
monolithic Flask/WebSocket app — the opposite of our modular ROS 2 approach.
