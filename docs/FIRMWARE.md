# The ESP32 "General Driver Board" — what the firmware actually does

Findings from reading [waveshareteam/ugv_base_general](https://github.com/waveshareteam/ugv_base_general)
and probing our board over serial (2026-07). This is the ground truth behind several
driver design decisions; read this before "fixing" anything speed- or IMU-related.
The reflash procedure that fixes the IMU is prepared in `docs/REFLASH.md`.

## Our board's state

* **REFLASHED 2026-07-02** to `ugv_base_general` v0.95 (`d308df9`) + our patch
  (`firmware/ugv-rover.patch`, procedure in `docs/REFLASH.md`; updated same day
  with the ICM20948 read path, see the IMU section). The board now boots with
  correct UGV Rover geometry compiled in, WiFi off, our verified motor
  direction. `{"T":1001}` carries `L,R,r,p,y,temp,v`. Direction, encoder
  scale (cmd 0.2 → meas ≈ 0.2), and the board dead-man (default 3s window,
  driver arms 1s) were verified wheels-off. The IMU works (0.2b, see below)
  but is **currently flashed OUT again** — its read path stalls the control
  loop; 0.2c restores it properly.
  The pre-flash firmware (which predated the public repo entirely — its frame
  lacked `y`) is preserved in `firmware/stock-*.bin` (local only, gitignored).
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

## The IMU: ICM20948, working (custom read path, 2026-07-02)

This board revision has **no QMI8658** (`WhoAmI` reads `0xff` on both its I2C
addresses) — it is the older revision carrying an **ICM20948 at I2C `0x68`**,
confirmed live. The historic "dead accel-Z / roll pins at 90°" garbage was a
chip/driver mismatch (stock pre-repo firmware reading the wrong chip), not a
broken sensor: accel-Z reads a healthy ±1g today.

The patch replaces the stock QMI8658+AK09918 stack in `IMU_ctrl.h` with an
ICM20948 read path (Adafruit driver, already a build dependency):

* **Runtime probe, not compile-time**: one bounded WHO_AM_I check at
  `0x68`/`0x69` at boot. Found → init + `imu: ICM20948 online at 0x68` on the
  console. Not found → the bus is never touched again (a missing chip's I2C
  timeouts would throttle the whole main loop — the original stock-firmware
  hang) and `r/p/y` stay `0.0` with `temp` = ESP32 internal.
* Reads at **100Hz** (the AK09916 mag's max; unthrottled reads would starve
  the ~1ms command loop), bus at 400kHz, feeding the stock Mahony fusion with
  two of its known bugs fixed: measured `dt` instead of the hardcoded 48ms
  period, and persistent (clamped ±20°/s) gyro-bias integral terms.
* **Verified on the bench** (tilt tests, all four poses + upside down): r/p
  track and hold with no drift, correct signs (positive pitch = nose down,
  positive roll = right side down), stable through the ±90° region.
* `temp` is the **IMU die temperature** (~37°C) — tracks ambient far better
  than the ESP32-internal fallback (~50°C).
* **Yaw is a relative heading, not compass truth**: converged yaw is stable
  (±1°), but the magnetometer is uncalibrated and reads ~2× earth field
  (hard iron from board/motors), and the fusion has no declination for our
  location. Fine for "how much did I turn", wrong as absolute north.
* `IMU_TRUSTED` is `true` in `webapp/app.js` since 2026-07-02.
* `{"T":126}` returns live `{"T":1002,...}` frames (mg / deg/s / µT);
  `{"T":127}` calibration remains a **no-op** (routine commented out at the
  source).

### STATUS 2026-07-03: board is back on the IMU-ABSENT build until 0.2c

The 0.2b read path was **convicted of stalling the control loop** by an A/B
`diag_latency` measurement (same conditions, only firmware differing):

|                    | 0.2 (IMU absent) | 0.2b (ICM reads @100Hz) |
|--------------------|------------------|--------------------------|
| cross-corr lag     | 85 ms            | 90–115 ms                |
| step median        | 73 ms            | 83–155 ms                |
| step worst-case    | **119 ms**       | **289–601 ms**           |

The median cost is modest but the **tail stalls (up to ~600 ms, plus a 330 ms
telemetry gap) freeze the control loop mid-drive** — felt as lag and drift.
Suspected mechanism: I2C contention on the shared bus (ICM20948 + SSD1306 +
INA219) escalating into Wire timeouts. So: `write_flash 0x10000` back to
`rover-v095-imuabsent-app0-2026-07-02.bin` (kept locally with the ICM build),
r/p/y are 0.0 again, temp = ESP32 internal, `IMU_TRUSTED` back to `false`.
**0.2c must fix BOTH problems** — the motor-interference gating below AND
moving IMU I/O off the control loop (dedicated FreeRTOS task on core 0 with a
mutex around the Wire bus it shares with the OLED and INA219 in the main
loop). Acceptance: `diag_latency` step worst-case back under ~150 ms with the
IMU active, attitude sane with motors running.

### Known limitation: attitude is garbage while the MOTORS run (task 0.2c)

Ground-observed 2026-07-02 (wheels-off, body stationary, wheels spinning):
r/p/y swing wildly even though the rover isn't moving. Two mechanisms, both
motor-driven:

1. **Vibration → accel noise.** The fusion trusts the accelerometer hard
   (Kp = 4.5); vibration makes the gravity estimate jump, dragging roll/pitch.
2. **Motor magnetic fields → mag garbage.** The mag already reads ~2× earth
   field at rest (hard iron); running motors add a large *varying* field. Mag
   error cross-couples into ALL axes in `imuAHRSupdate` (and can wind the
   ±20°/s bias integrals, so garbage persists briefly after motors stop).

So: attitude is trustworthy **at rest only** until fixed. The fix is firmware
sample gating in `IMU_ctrl.h`/`IMU.cpp` (sketch-only → app0-only reflash):

* skip the accel correction when `| |a| − 1g |` exceeds a window (pass gyro-only
  integration for that sample) — standard vibration rejection;
* skip the mag correction when mag magnitude leaves a band around its at-rest
  baseline (learned at boot) — standard motor-interference rejection;
* consider Kp ≈ 1.0 and a smaller integral clamp while at it;
* verify on the bench: readings must stay put with wheels spinning at various
  speeds, still track real tilts, and recover instantly when motors stop.

## Historical: the old firmware's IMU readings (pre-reflash)

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
