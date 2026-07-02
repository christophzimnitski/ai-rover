# Reflashing the ESP32 (Phase 0.2)

Why: our board runs a firmware build that **predates the public
[`ugv_base_general`](https://github.com/waveshareteam/ugv_base_general) repo**
(its `{"T":1001}` frame has no `y`/`temp` field — even the repo's first commit had `y`).
That old build reads the IMU wrong at the source (dead accel-Z, no magnetometer →
roll pins at 90°; see `docs/FIRMWARE.md`). The current firmware drives the board's
QMI8658 + AK09918 IMU natively, re-enables the magnetometer read, and adds
`y` (yaw) + `temp` to the feedback frame.

## What to flash

**`ugv_base_general` @ `d308df91b333a513f45a238bef9ba9a0a5edf64c`** ("version: 0.95",
2025-11-28 HEAD) **plus [`firmware/ugv-rover.patch`](../firmware/ugv-rover.patch)** —
build from source.

**Do NOT use the repo's prebuilt `General_Driver/build/` binaries.** They were built
at commit `b113287` ("version: 0.90") and are missing the two later source commits
that matter: `bc2184c` re-enables the magnetometer read and adds `y`+`temp` to the
frame. The prebuilt bin would leave us with roughly what we already have.

### Why each line of the patch (3 lines, all compile-time defaults)

The stock firmware calls `mm_settings(mainType, moduleType)` in `setup()`, which
**overrides** the geometry variables. There is no stock configuration that is
correct for our rover:

| stock option | what you get | why it's wrong for us |
|---|---|---|
| boot default (`mainType=1`, RaspRover) | `ONE_CIRCLE_PLUSES=2100`, `TRACK_WIDTH=0.125`, dir normal | encoder speeds read ~21% low (real scale is 1650), wrong track width |
| `{"T":900,"main":2}` (UGV Rover) | `1650` / `0.172` (correct), but `SET_MOTOR_DIR=true` | **flips drive + feedback sign** vs. our ground-verified wiring |

The patch (applies cleanly to `d308df9` with `git apply`):

1. `ugv_config.h`: `byte mainType = 1;` → `2` — boots as UGV Rover: correct
   encoder scale (1650 pulses) and track width (0.172). OLED will show "UGV02 Null".
2. `movtion_module.h` (`mm_settings`, `mainType==2` branch): `SET_MOTOR_DIR = true;`
   → `false` — keeps our ground-verified drive/feedback direction. If the wheels-off
   test below shows reversed motion, flip this back and rebuild (never "fix" a
   reversal in the Pi driver with sign hacks).
3. `wifi_ctrl.h`: `byte WIFI_MODE_ON_BOOT = 1;` → `0` — the stock default would
   broadcast an open AP (`UGV` / `12345678`) with a web control page on every boot.
   Off; can still be enabled over UART later if ever wanted.

The **"never send `{"T":900}`" rule still stands** after reflashing: everything is
compiled in; a runtime `T:900` with `main:1` would silently load the wrong geometry
again (it does not persist, but why gamble).

## Build environment (pin these)

Known-good versions taken from the vendor's own build artifacts (`.map` file):

- **ESP32 Arduino core `2.0.14`** (the sketch is written against core 2.x APIs —
  do not use 3.x)
- Board FQBN: `esp32:esp32:esp32` ("ESP32 Dev Module"), default 4MB partition
  scheme (app is ~995KB, fits), upload speed default
- **ArduinoJson 6.x** (e.g. 6.21.5) — the sketch uses the v6 `StaticJsonDocument`
  API; v7 deprecated it
- Library Manager: `Adafruit SSD1306`, `INA219_WE`, `ESP32Encoder`, `PID_v2`,
  `SimpleKalmanFilter`, `Adafruit ICM20X` (+ its `Adafruit ICM20948` /
  `Adafruit Sensor` dependencies — still `#include`d even though the ICM chip
  driver is unused on this board rev). `LittleFS` is bundled with core 2.x.
- **SCServo**: not in the Library Manager — copy the `SCServo/` folder from the
  `ugv_base_general` repo into the Arduino `libraries/` directory.

### arduino-cli one-time setup + build (Mac)

```bash
brew install arduino-cli
arduino-cli config init --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32@2.0.14
arduino-cli lib install "ArduinoJson@6.21.5" "Adafruit SSD1306" "INA219_WE" \
    "ESP32Encoder" "PID_v2" "SimpleKalmanFilter" "Adafruit ICM20X"

git clone https://github.com/waveshareteam/ugv_base_general.git
cd ugv_base_general
git checkout d308df91b333a513f45a238bef9ba9a0a5edf64c
git apply /path/to/this-repo/firmware/ugv-rover.patch
cp -r SCServo ~/Documents/Arduino/libraries/

arduino-cli compile --fqbn esp32:esp32:esp32 General_Driver
```

## Flashing

1. **Prep the rover**: put it on a stand — **wheels off the ground** (they stay off
   until the checklist below passes). Stop the driver so nothing talks on the UART
   during/after the flash: `ssh rover 'cd ~/rover/docker && docker compose stop rover'`.
2. **Connect USB**: the driver board's ESP32 **USB-C port** to the Mac (this is the
   UART0/download port with auto-reset — not the LiDAR port). Board can stay
   battery-powered. Find the port: `ls /dev/cu.usbserial*` (CP210x) or `/dev/cu.wchusbserial*`.
3. **Full flash erase first** (guarantees: empty boot mission, no stored WiFi
   config, no stale LittleFS) — esptool ships inside the ESP32 core:
   ```bash
   python3 ~/Library/Arduino15/packages/esp32/tools/esptool_py/*/esptool.py \
       --chip esp32 --port /dev/cu.usbserial-XXXX erase_flash
   ```
4. **Upload**:
   ```bash
   arduino-cli upload -p /dev/cu.usbserial-XXXX --fqbn esp32:esp32:esp32 General_Driver
   ```

## Post-flash verification (SAFETY-CRITICAL — wheels off the ground)

Run from the **Pi host** (rover compose service still stopped, port is
`/dev/ttyAMA0` on the host — never `/dev/serial0`, see `docs/COMMANDS.md`).

1. **Boot screen**: OLED shows `AP: OFF` / `ST: OFF`, `UGV02 Null`, `version: 0.95`.
   Anything else = wrong binary/patch, stop here.
2. **IMU sanity** (`{"T":126}`; the board answers with a `{"T":1002,...}` frame —
   units are mg and deg/s):
   - flat: `|az| ≈ 1000`, `ax`,`ay` ≈ 0 (±100), `r`,`p` ≈ 0 (±5°), gyro ≈ 0 (±3),
     `mx,my,mz` non-zero and changing when the rover is rotated
   - tilt the rover ~30° and hold: `r`/`p` follow and are stable (no drift to 90°)
   - a `qmi8658_init fail` on the serial console or all-zero readings = the board
     is an older IMU revision; the reflash still stands (drive/telemetry unaffected)
     but roll/pitch stay untrusted — do NOT flip `IMU_TRUSTED`.
3. **Feedback frame**: `{"T":1001}` frames now carry `L,R,r,p,y,temp,v`.
4. **Drive direction + scale** (THE critical test, wheels OFF the ground):
   ```bash
   python3 ~/rover/scripts/serial_smoketest.py --port /dev/ttyAMA0 --left 0.2 --right 0.2 --duration 1.5
   ```
   - all wheels spin **forward**, feedback `L ≈ R ≈ +0.2` (confirms 1650-pulse scale;
     ~0.157 would mean RaspRover 2100 geometry got loaded — patch didn't take)
   - reversed wheels ⇒ flip `SET_MOTOR_DIR` to `true` in the patch, rebuild,
     reflash — do not compensate in the driver
   - then `--left -0.2 --right 0.2`: left wheels backward, right forward
5. **Dead-man**: with the stream stopped mid-motion (Ctrl-C the smoketest while
   wheels spin), motors stop on their own within the heartbeat window.
6. **Full stack**: `docker compose start rover`, then check the dashboard:
   `/rover/telemetry` at 20Hz with live `roll/pitch/yaw/temp`, gamepad drives
   (wheels still off the ground), e-stop latches.
7. Only now: wheels on the ground, short manual drive (straight line + spin), watch
   for anything odd in trim behavior (`wheel_trim` assumptions unchanged: open-loop
   PWM + real encoder feedback — both preserved by this firmware for `mainType` 1/2).

## After verification passes

- Flip `IMU_TRUSTED = true` in `webapp/app.js` (drops the dashboard warning badge).
  Caveat that stays documented: yaw is fused with hardcoded magnetometer offsets and
  a hardcoded (Shenzhen) declination — treat yaw as relative heading, not compass truth.
- Update `docs/FIRMWARE.md`: board now on 0.95-patched, frame carries `y`+`temp`,
  IMU section becomes historical. Note `{"T":127}` (IMU calibration) is a **no-op**
  in this firmware — the routine is commented out at the source; drop it from checklists.
- Re-run `scripts/diag_latency.py` to re-baseline (roadmap 0.3 feeds off this).
