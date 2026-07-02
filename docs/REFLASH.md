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

> **OUTCOME (2026-07-02): flashed and verified.** One major discovery during the
> flash: **this board has no QMI8658** — `WhoAmI` reads `0xff` on both I2C
> addresses. It is the older board revision with an **ICM20948 at 0x68**
> (confirmed live later the same day). The stock firmware hangs its main loop
> on the missing QMI8658's I2C timeouts, so the patch first compiled the IMU
> out (`IMU_ABSENT`), then grew the proper fix the same day (**task 0.2b**): a
> custom ICM20948 read path in `IMU_ctrl.h` feeding the stock AHRS fusion —
> runtime-probed, so a board without the chip degrades to the IMU-absent
> behavior instead of hanging. Bench tilt tests verified roll/pitch/yaw
> (details + caveats in `docs/FIRMWARE.md`). The 0.2b flash was an
> **app0-only write** (`write_flash 0x10000` — no erase, bootloader/partitions/
> LittleFS untouched), which is the cheap path for any future sketch-only change.

### Why each line of the patch (all compile-time defaults)

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
4. `IMU_ctrl.h` (+ `IMU.h`/`IMU.cpp`): the QMI8658+AK09918 stack is replaced by
   an **ICM20948 read path** — this board revision has no QMI8658 (see outcome
   note above), and without a guard the missing chip's I2C timeouts throttle
   the entire command loop to uselessness. The chip is probed once at boot
   (bounded WHO_AM_I at 0x68/0x69): found → 100Hz reads into the stock Mahony
   fusion (with measured `dt` and clamped persistent bias integrals); not
   found → the bus is never touched again and r/p/y stay 0.0. Details in
   `docs/FIRMWARE.md`.

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
- **INA219_WE 1.3.x** (e.g. 1.3.8) — 1.4+ renamed the constants (`BIT_MODE_9`,
  `PG_320`, `BRNG_16` → `INA219_*`) and the sketch uses the old names
- Library Manager: `Adafruit SSD1306`, `ESP32Encoder`, `PID_v2`,
  `SimpleKalmanFilter`, `Adafruit ICM20X` (+ its `Adafruit ICM20948` /
  `Adafruit Sensor` dependencies — still `#include`d even though the ICM chip
  driver is unused on this board rev). `LittleFS` is bundled with core 2.x.
- **Apple Silicon Mac**: the core's bundled `ctags`/`esptool` are Intel-only —
  install Rosetta 2 first (`softwareupdate --install-rosetta --agree-to-license`).
- **SCServo**: not in the Library Manager — copy the `SCServo/` folder from the
  `ugv_base_general` repo into the Arduino `libraries/` directory.

### arduino-cli one-time setup + build (Mac)

```bash
brew install arduino-cli
arduino-cli config init --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32@2.0.14
arduino-cli lib install "ArduinoJson@6.21.5" "Adafruit SSD1306" "INA219_WE@1.3.8" \
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
   battery-powered. The port shows up as `/dev/cu.usbmodem*` (CDC) or
   `/dev/cu.usbserial*` depending on bridge chip. Chip is an **ESP32-D0WD-V3**.
3. **Release the Pi's UART TX** — the board's USB-C and the Pi GPIO header share
   the ESP32's UART0. The Pi's TX (GPIO14) overpowers the USB bridge even when
   nothing is transmitting ("serial TX path seems to be down" from esptool):
   ```bash
   ssh rover 'pinctrl set 14 ip'    # tri-state TX; restore after: pinctrl set 14 a4
   ```
4. **esptool quirks on macOS** (hard-won): the macOS CDC driver has **no flow
   control** to this bridge chip — any bulk write faster than the UART line rate
   overflows the bridge FIFO and corrupts data (stub upload fails "Failed to
   write to target RAM"; `--no-stub` writes fail too, and no-stub can't
   `erase_flash` or read above 2MB). The fix is `firmware/esptool_chunked.py`:
   a wrapper that paces serial writes to the wire rate (64-byte chunks,
   `len*10/baud*1.2` sleep). With it the stub runs and everything works,
   including compressed+verified writes at `--baud 460800`.
   Flashing **from the Pi** avoids all of this (Linux driver is fine) — it just
   needs a *data* USB-A→C cable; charge-only cables don't enumerate at all.
5. **Back up the current firmware first** — done 2026-07-02, kept locally as
   `firmware/stock-fw-backup-first2MB-esp32-2026-07-02.bin` (bootloader +
   partition table + full active app0; the upper 2MB is only the inactive OTA
   slot + SPIFFS) and `firmware/stock-app0-esp32-2026-07-02.bin` (the exact
   stock app image — restore with `write_flash 0x10000 <file>`). Backups stay
   local, `firmware/*.bin` is gitignored. For a fresh backup:
   ```bash
   ESPTOOL='<venv>/bin/python3 firmware/esptool_chunked.py'   # macOS; plain esptool.py elsewhere
   $ESPTOOL --baud 460800 --port /dev/cu.usbmodemXXXX \
       read_flash 0 0x400000 firmware/stock-fw-backup-esp32-<date>.bin
   ```
6. **Full flash erase** (guarantees: empty boot mission, no stored WiFi config,
   no stale LittleFS):
   ```bash
   $ESPTOOL --port /dev/cu.usbmodemXXXX erase_flash
   ```
7. **Write** (offsets per ESP32 Arduino convention):
   ```bash
   $ESPTOOL --baud 460800 --port /dev/cu.usbmodemXXXX write_flash \
       0x1000 fw-build/General_Driver.ino.bootloader.bin \
       0x8000 fw-build/General_Driver.ino.partitions.bin \
       0xe000 ~/Library/Arduino15/packages/esp32/hardware/esp32/2.0.14/tools/partitions/boot_app0.bin \
       0x10000 fw-build/General_Driver.ino.bin
   ```
   esptool verifies each image by hash. Note: opening the serial port afterwards
   pulses DTR/RTS and **reboots the board** — wait ~10s for boot ("missionPlay
   finished" on the console) before sending commands.
8. **Restore the Pi's UART TX**: `ssh rover 'pinctrl set 14 a4'`.

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
   - the boot console prints the probe verdict: `imu: ICM20948 online at 0x68`
     vs `imu: no ICM20948 at 0x68/0x69, IMU disabled`. Disabled = all-zero
     r/p/y, drive/telemetry unaffected — do NOT flip `IMU_TRUSTED`.
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
