#!/usr/bin/env python3
"""Minimal, ROS-free hardware check for the Waveshare UGV Rover.

Streams a wheel-speed command to the ESP32 over serial, reads the board's
feedback (battery voltage + measured left/right wheel speed), then stops. Run
this FIRST to prove the serial link, power and motors work before bringing up
ROS 2. It always sends a stop on exit, even on Ctrl-C.

Why stream instead of one command: the stock app drives by re-sending commands
continuously, and a single {"T":1} may only twitch the wheels. This matches how
our ROS driver drives (teleop_twist_keyboard with repeat_rate).

Port: on a Raspberry Pi 5 the ESP32 is on /dev/ttyAMA0 (GPIO14/15 PL011).
Do NOT use /dev/serial0 on the host here — it is aliased to /dev/ttyAMA10, a
different, unconnected UART. (Inside the container /dev/serial0 is correct: compose
maps host ttyAMA0 -> /dev/serial0.)

Examples:
    # nudge both wheels forward at 0.2 m/s for 1.5 s, then stop
    python3 serial_smoketest.py --port /dev/ttyAMA0 --left 0.2 --right 0.2 --duration 1.5

    # spin in place (left wheels back, right wheels forward)
    python3 serial_smoketest.py --left -0.2 --right 0.2 --duration 1.0
"""
import argparse
import json
import sys
import time

try:
    import serial  # pyserial
except ImportError:
    sys.exit("pyserial not installed. Inside the container it is preinstalled; "
             "on the Pi host run: sudo apt install python3-serial")

# ESP32 "General Driver Board" JSON commands (newline-delimited).
CMD_SPEED = 1          # {"T":1,"L":<mps>,"R":<mps>}  wheel speed control
CMD_FEEDBACK_FLOW = 131  # {"T":131,"cmd":1}  turn serial telemetry stream on/off
CMD_FEEDBACK_INT = 142   # {"T":142,"cmd":<ms>}  telemetry interval
FEEDBACK_TYPE = 1001     # {"T":1001,"L":..,"R":..,"v":volts,...}  telemetry reply


def send(ser, obj: dict) -> None:
    ser.write((json.dumps(obj) + "\n").encode())


def read_feedback(ser):
    """Return the latest {"T":1001} telemetry dict waiting in the buffer, or None."""
    latest = None
    while ser.in_waiting:
        line = ser.readline()
        if not line:
            break
        try:
            d = json.loads(line.decode("utf-8", "ignore"))
        except ValueError:
            continue
        if isinstance(d, dict) and d.get("T") == FEEDBACK_TYPE:
            latest = d
    return latest


def main() -> None:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default="/dev/ttyAMA0")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--left", type=float, default=0.2)
    p.add_argument("--right", type=float, default=0.2)
    p.add_argument("--duration", type=float, default=1.5)
    p.add_argument("--rate", type=float, default=20.0, help="command stream Hz")
    p.add_argument("--no-feedback", action="store_true",
                   help="don't request/print ESP32 telemetry")
    args = p.parse_args()

    for v in (args.left, args.right):
        if not -0.5 <= v <= 0.5:
            sys.exit("left/right must be within the ESP32 limit of -0.5..+0.5 m/s")

    print(f"Opening {args.port} @ {args.baud} ...")
    ser = serial.Serial(args.port, args.baud, timeout=0.2)
    last = None
    try:
        if not args.no_feedback:
            send(ser, {"T": CMD_FEEDBACK_INT, "cmd": 50})
            send(ser, {"T": CMD_FEEDBACK_FLOW, "cmd": 1})
            time.sleep(0.4)
            last = read_feedback(ser)
            v = last.get("v") if last else None
            print(f"link: {'OK' if last else 'NO TELEMETRY (check port/wiring)'}"
                  + (f"  battery={v:.2f} V" if isinstance(v, (int, float)) else ""))

        print(f"streaming L={args.left} R={args.right} for {args.duration}s "
              f"@ {args.rate}Hz ...")
        period = 1.0 / args.rate
        end = time.time() + args.duration
        while time.time() < end:
            send(ser, {"T": CMD_SPEED, "L": round(args.left, 4), "R": round(args.right, 4)})
            time.sleep(period)
            fb = read_feedback(ser)
            if fb:
                last = fb
        if last and not args.no_feedback:
            print(f"echo: measured L={last.get('L')} R={last.get('R')} "
                  f"v={last.get('v')}")
    finally:
        send(ser, {"T": CMD_SPEED, "L": 0.0, "R": 0.0})   # always stop
        if not args.no_feedback:
            send(ser, {"T": CMD_FEEDBACK_FLOW, "cmd": 0})
        ser.close()
        print("stopped, port closed.")


if __name__ == "__main__":
    main()
