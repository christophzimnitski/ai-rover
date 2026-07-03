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

Known constraints: wheel speed feedback quantized ~0.15 m/s (re-measure in 0.3);
controller-runaway on dongle drop is accepted (mitigated by e-stop + board
heartbeat); IMU yaw is a relative heading, not compass truth (uncalibrated mag).

## Phase 0 — Hardware foundation (do first)

- **0.1 Replace the failing microSD — Done** (or migrate to USB-SSD). Reflash Pi OS,
  run `scripts/bootstrap.sh`, re-apply the camera udev rule — and commit that
  rule into `bootstrap.sh`/`PI_SETUP.md` so a fresh Pi gets it automatically.
- **0.2 Reflash the ESP32 — DONE 2026-07-02** (`docs/REFLASH.md`): flashed
  `ugv_base_general` v0.95 (`d308df9`) + `firmware/ugv-rover.patch` (UGV Rover
  geometry at boot, our motor direction, WiFi AP off, IMU compiled out).
  Verified wheels-off: direction, encoder scale, dead-man, `y`/`temp` in
  telemetry, full stack at 20Hz. **Discovery: this board has no QMI8658** (older
  rev with an ICM20948 at 0x68) — the old "broken IMU" was a chip/driver
  mismatch, not a broken sensor.
- **0.2b Custom ICM20948 read path — DONE 2026-07-02**: the patch's IMU
  compile-out became a runtime-probed ICM20948 driver feeding the stock AHRS
  fusion (probe fails safe: no chip → no bus traffic, r/p/y = 0.0). Bench
  tilt tests verified roll/pitch/yaw signs, tracking, and stability;
  `IMU_TRUSTED` flipped true. Yaw = relative heading only (uncalibrated mag);
  details in `docs/FIRMWARE.md`.
- **0.2c Restore the IMU without hurting the control loop** — the 0.2b read
  path was A/B-measured to stall the ESP32 loop (step worst-case 289–601 ms
  vs 119 ms without it; suspected shared-I2C contention), so the board is
  back on the IMU-absent build. Two fixes, one task: (1) move IMU I/O to a
  core-0 task with a Wire mutex (bus shared with OLED + INA219), (2) gate
  the fusion against motor vibration/magnetic interference. Acceptance:
  `diag_latency` step worst-case < ~150 ms with IMU active + attitude sane
  with motors running. Full design in `docs/FIRMWARE.md`.
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
