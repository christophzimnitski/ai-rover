# The ESP32 "General Driver Board" — what the firmware actually does

Findings from reading [waveshareteam/ugv_base_general](https://github.com/waveshareteam/ugv_base_general)
and probing our board over serial (2026-07). This is the ground truth behind several
driver design decisions; read this before "fixing" anything speed- or IMU-related.

## Our board's state

* Firmware is an **older build** of `ugv_base_general`: its `{"T":1001}` feedback frame
  carries only `L,R,r,p,v` (current firmware adds `y` = yaw and `temp`).
* The **boot mission is empty** and nothing sends `{"T":900,...}` (robot-type select)
  since the stock app was disabled, so the board runs its compiled-in defaults:
  `mainType=1`, geometry `WHEEL_D=0.08 / 1650 pulses / TRACK_WIDTH=0.172` — which
  happens to match the UGV Rover, so the encoder speed scale in telemetry is correct.
* **Do NOT send `{"T":900,"main":2}`**: `mm_settings()` would flip `SET_MOTOR_DIR`,
  reversing drive + feedback signs. Direction is ground-verified correct as-is.

## Speed control is OPEN LOOP (the friction/lurch explanation)

`{"T":1,"L":x,"R":x}` runs `setGoalSpeed()`, and with our `mainType` that is
`leftCtrl(x * 512)` — a straight **PWM write** saturating at 255:

* the "m/s" in the command is really a throttle fraction; `0.498 → PWM 255` is why
  nothing changes above **0.5** ("max_wheel_speed"),
* the same command gives different true speeds per surface, and low commands stall
  against static friction until it breaks, then the rover jumps (kinetic friction is
  lower). Nothing in the firmware "catches up" — it's open-loop + stiction physics.
* The firmware's PID (`{"T":2}`, `usePIDCompute`) only activates for `mainType==3`
  (UGV01/tracked); it is **dead code for our type** on every firmware version.

**Our fix** is Pi-side: `ugv_driver`'s `wheel_trim` (kinematics.SpeedTrim) closes the
loop with the encoder feedback the board does send — a slow per-wheel integrator makes
commanded m/s physically true and breaks stiction smoothly. `min_wheel_speed` is the
legacy open-loop workaround; keep it 0 while trim is on.

* Feedback `L/R` **are** real encoder speeds, but computed over one ~1ms main-loop
  iteration → quantized to ~**0.15 m/s** steps. Filtering (trim's EMA) recovers
  usable resolution; per-pulse odometry would need firmware changes.

## The IMU readings are broken at the source (roll=90°)

Probed flat on the bench, `{"T":126}` returns:

```
r=90.00  p=-46.77  ax=-82.1  ay=77.2  az=-3e-7  gz=-20.8  mx,my≈0  mz=3021
```

* `az ≈ 0` (should be ±1g flat), `mx,my ≈ 0`, `gz = -20.8°/s while stationary.
  With a dead accel Z the Mahony fusion collapses → roll pins at exactly 90°.
* The fusion code itself is also buggy on every version (integral terms reset each
  call, hardcoded sample period, magnetometer fused with hardcoded Shenzhen
  declination + default offsets). Waveshare's own commit log: "imu mag bug, ignored
  for now".
* **No Pi-side math can rescue r/p from this data.** The real fix is reflashing the
  current `ugv_base_general` (Arduino IDE or Waveshare's flash tool), which also adds
  yaw + temp to the feedback frame. After reflashing:
  1. verify drive direction and telemetry signs (type defaults may differ!),
  2. run `{"T":127}` IMU calibration flat + still,
  3. flip `IMU_TRUSTED = true` in `webapp/app.js` to drop the dashboard warning.
  The driver already publishes `yaw`/`temp` (0.0 until the firmware provides them).

## Board features we now use (were unused before)

| T-code | What | Our seam |
|-------:|------|----------|
| 132 | headlight PWM (IO4/IO5) | `/rover/lights` (ugv_msgs/Lights) |
| 3 / -3 | OLED text / default screen | `/rover/oled` (ugv_msgs/OledText) |
| 136 | board-side dead-man (stops motors if no cmd for N ms) | `board_heartbeat_ms` param, sent at startup + every link re-open |
| 131/142 | feedback stream on/off + interval | telemetry (already used) |

Still available, deliberately unused: `{"T":11}` raw PWM, `{"T":13}` board-side twist
mix (uses the board's TRACK_WIDTH, not ours), `{"T":138/140}` persisted per-side speed
scale, `{"T":126..129}` raw IMU poll + calibration (garbage until reflash), missions/
files in flash, ESP-NOW, wifi (board AP stays off), `{"T":600}` reboot.

## Serial protocol notes

* 115200 baud, newline-delimited JSON both ways. Only `{"T":1001}` frames are
  telemetry; everything else (echoes, acks, `{"T":1002}` IMU replies) must be filtered.
* The board **forgets everything on reset/brownout** (stream arming, heartbeat window,
  lights). `SerialLink` re-arms the stream and fires `on_link_up` so the driver
  re-sends config after every recovery.
* Field-observed failure (2026-07): after ~3h the feedback stream died with a healthy-
  looking fd; re-arming alone didn't help, close+reopen did. `SerialLink` now
  escalates: stale → re-arm → still stale → full fd re-open.
