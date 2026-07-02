# Rover roadmap

Living plan for the Waveshare UGV Rover. End goal: **full parity with the stock
Waveshare app** (video, driving, lights, OLED, media capture, vision features)
rebuilt on our modular ROS 2 stack — plus what the stock app never had: pluggable
custom vision models and a **control panel where every feature can be enabled,
disabled, and tuned**. The rover ultimately recognizes and follows the cat.

Architecture rules stand: every concern is its own independent service behind a
typed seam; `ugv_driver` is the only thing speaking the ESP32's serial JSON; the
webapp stays a thin zero-dependency layer (display + forwarding user intent);
heavy work (vision, media, logging) lives in separate services.

## Where we are (2026-07-02)

Done: driver + gamepad teleop (self-healing), typed telemetry seam, camera
service (exposure pinned), webbridge, dashboard v2 (video, telemetry, drive pad,
e-stop, lights, OLED, link health), Pi-side speed loop (`wheel_trim`) over the
open-loop firmware, board heartbeat armed, video path single-encode
(`ros_compressed`, q60). Firmware ground truth documented in `docs/FIRMWARE.md`.

Known constraints: board IMU is broken at the source until reflashed (roll pins
at 90°); wheel speed feedback quantized ~0.15 m/s; controller-runaway on dongle
drop is accepted (mitigated by e-stop + board heartbeat); the microSD is failing.

## Phase 0 — Hardware foundation (do first)

- **0.1 Replace the failing microSD** (or migrate to USB-SSD). Reflash Pi OS,
  run `scripts/bootstrap.sh`, re-apply the camera udev rule — and commit that
  rule into `bootstrap.sh`/`PI_SETUP.md` so a fresh Pi gets it automatically.
- **0.2 Reflash the ESP32** — procedure prepared, see **`docs/REFLASH.md`**:
  build `ugv_base_general` @ `d308df9` (v0.95) + `firmware/ugv-rover.patch`
  (3 lines: UGV Rover geometry at boot, keep our motor direction, WiFi AP off).
  Fixes the IMU (native QMI8658+AK09918 drivers) and adds yaw+temp to feedback.
  Prebuilt repo bins are stale (v0.90) — source build only. Post-flash checklist
  is safety-critical (wheels off ground: direction + feedback signs, `{"T":126}`
  sanity with |az| ≈ 1000mg flat, dead-man). `{"T":127}` calibration is a no-op
  in this firmware — dropped. Then flip `IMU_TRUSTED` in `webapp/app.js` and
  drop the dashboard warning badge.
- **0.3 Ground-tune `wheel_trim`** on tile + carpet (trim_ki / trim_limit);
  re-measure with `scripts/diag_latency.py`.

## Phase 1 — Control-panel foundation (feature flags)

- **1.1 Design the feature registry.** Convention: every feature is owned by one
  service and exposed as ROS parameters (`enabled` + tuning knobs), discoverable
  and settable from the browser via rosapi service calls over the existing
  rosbridge (zero new dependencies). A common `ServiceStatus` heartbeat topic
  per service (generalizing `/rover/link`).
- **1.2 Parameterize existing nodes** (driver: trim/mux/heartbeat knobs; camera:
  exposure values) so the panel can control them live.
- **1.3 Webapp Control tab**: registry-driven list of every feature with
  toggle/slider/select, live get/set via rosbridge; explicit **persist** button
  backed by a small config-writer node that updates the bind-mounted yamls.
- **1.4 Service-level on/off**: compose profiles documented; the panel shows
  per-service health (it does not control Docker by design).

## Phase 2 — Stock-app parity: non-vision features

- **2.1 `system` service**: `/rover/system` @1Hz (CPU, RAM, temp, disk, WiFi
  RSSI, uptime) from `/proc`+`/sys`, zero-dep; dashboard panel; calibrated 3S
  battery %.
- **2.2 `media` service**: photo capture + video recording from
  `/camera/image_raw/compressed`, gallery served over its own HTTP port,
  storage on the (new) SSD; webapp Gallery tab with record/photo buttons.
- **2.3 OLED status rotation** (IP / battery / uptime) as a panel-toggleable
  feature of the system service.
- **2.4 Audio interaction** — optional, only if speaker hardware is added.

## Phase 3 — Vision platform (models, including your own)

- **3.1 Design the detection seam**: `vision_msgs/Detection2DArray` on
  `/rover/detections`; a `models/<name>/manifest.json` convention (framework,
  input size, labels, thresholds) so custom models drop in without code changes.
  Pick the minimal runtime set (OpenCV + one inference runtime).
- **3.2 `ugv_perception` skeleton**: image in → pluggable detector → detections
  out + FPS/status; panel controls (enable, model picker, confidence, input
  size, frame skip).
- **3.3 Dashboard overlay**: draw boxes/labels on the existing `<canvas
  id="overlay">`; per-class colors; panel toggle.
- **3.4 Parity detectors, one at a time**: motion detection (no model), color
  tracking (HSV, panel color picker), face detection, object detection
  (includes the cat), gesture recognition (optional), line detection (optional).
- **3.5 Custom-model onboarding docs** + verify with one non-stock model.

## Phase 4 — Behaviors (autonomy)

- **4.1 Formalize the command mux** in the driver: e-stop > gamepad `/cmd_vel` >
  `/web_cmd_vel` > `/behavior_cmd_vel`, per-source enable in the panel, and an
  explicit "autonomy armed" latch (panel switch, OLED indication).
- **4.2 Follow-target behavior**: detections → bearing/size P-controller →
  `/behavior_cmd_vel`; target class, gains, speed caps, lost-target stop — all
  panel-tunable. This is the cat-following milestone.
- **4.3 Line-follow behavior** (optional stock parity).
- **4.4 Safety validation session**: e-stop drills, watchdog/runaway tests,
  heartbeat pull-the-plug test — before any unattended autonomy.

## Phase 5 — Polish & ops

- **5.1 Webapp reorg into tabs** (Drive / Control / Gallery / System), still
  zero-dependency.
- **5.2 Untethered hardening**: Tailscale access, low-battery safe-stop behavior
  (auto-halt + OLED + light blink), boot-time health OLED line.
- **5.3 Latency check on WiFi roaming**; evaluate WebRTC only if MJPEG falls
  short.
- **5.4 Docs pass** (`PI_SETUP.md`, `COMMANDS.md`, fresh-Pi restore drill).

## Reference: the stock Waveshare app (`~/ugv_rpi`, read-only)

Keep as documentation only — do **not** edit/delete it. It holds the confirmed
T-code protocol usage and the stock feature list we are matching. Its crontab
autostart stays disabled so it never contends for the UART/camera.
