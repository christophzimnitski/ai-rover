# Pi setup — do this once, on the Pi, then report back

This is the only part I can't do for you remotely: the Raspberry Pi currently has
SSH off (probably) and I don't know its address. Do the steps below on the Pi
desktop you already have open, then send me the few facts at the bottom and I'll
take over from the Mac.

---

## 1. Enable SSH + the serial port

Open a **Terminal** on the Pi desktop and run:

```bash
sudo raspi-config
```

- **Interface Options → SSH → Enable.**
- **Interface Options → Serial Port:**
  - "Would you like a login shell to be accessible over serial?" → **No**
  - "Would you like the serial port hardware to be enabled?" → **Yes**

Finish and reboot if prompted. (GUI alternative: *Preferences → Raspberry Pi
Configuration → Interfaces* → SSH **on**, Serial Port **on**, Serial Console **off**.)

> Why "login shell over serial = No": that console would fight our driver for the
> same UART the ESP32 is wired to (GPIO14/15). It must be off.

## 2. Gather the facts I need

Run these and keep the output:

```bash
uname -m                          # expect: aarch64   (64-bit is required for ROS 2)
cat /proc/device-tree/model       # which Pi (4B / 5)
grep PRETTY /etc/os-release       # which OS
ls -l /dev/serial0                # note what it points to: ttyAMA0 or ttyS0
hostname -I                       # the Pi's IP, e.g. 192.168.1.42
hostname                          # for <name>.local mDNS access
whoami                            # your Pi username
```

## 3. Install Docker (one time)

```bash
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker "$USER"
sudo reboot           # needed so your user picks up the 'docker' group
```

After it reboots, verify:

```bash
docker run --rm hello-world
docker compose version
```

## 4. Recommended long-term settings (optional, but worth doing now)

You're already in the Pi — these make a roaming, headless, vision-running robot reliable.

**Identity & network — so `ssh rover` always works:**
- Hostname: `sudo raspi-config` → System Options → Hostname → e.g. `rover`. Then `ssh rover.local`.
- Fixed address: easiest is a **DHCP reservation** in your router for the Pi's WiFi MAC
  (alternative: a static IP on the Pi). A robot whose IP keeps changing is painful.
- WiFi country: raspi-config → Localisation → WLAN Country (correct = legal + better signal).
- Kill WiFi power-saving (it adds lag / dropouts when driving live):
  `sudo iw dev wlan0 set power_save off` (we'll make it permanent via a config drop-in later).

**Rock-solid serial (this is a serial-controlled robot):**
- Depends on what `ls -l /dev/serial0` shows. If it points to **ttyS0** (the "mini-UART",
  whose baud rate drifts with CPU clock), we'll add `dtoverlay=disable-bt` to
  `/boot/firmware/config.txt` so the GPIO uses the better PL011 UART. If it points to
  **ttyAMA0**, you're already good. Don't change anything yet — just send me what it shows.

**Keep it healthy:**
- Update once: `sudo apt update && sudo apt full-upgrade -y`, then firmware `sudo rpi-eeprom-update -a`.
- Cooling: a heatsink/fan matters — the Pi throttles under load, and cat-detection will push it.
- RAM headroom for models: we'll enable zram / more swap when we add the vision milestone.
- Security: once SSH keys work, we'll disable SSH password login (it's a networked device).

## 5. (Optional but recommended) Confirm the motors work with the stock app

If Waveshare's stock software is still installed, point a browser (on the Pi or
your Mac) at `http://<PI_IP>:5000` and try the on-screen drive controls. If the
wheels move, the hardware is healthy and any later problem is in our stack, not
the robot.

> Heads-up: the stock service holds the serial port open. Before we run our own
> driver we'll stop it (we'll find the exact service name together — likely a
> systemd service or an autostart entry from Waveshare's `setup.sh`).

---

## 6. Report back to me

Paste me the output of step 2, plus your Pi **username** and **IP** (or hostname).
With that I will, from your Mac:

1. set up passwordless SSH (`ssh rover`),
2. sync this repo to the Pi,
3. build + launch the ROS 2 driver in Docker,
4. and we drive it with WASD.
