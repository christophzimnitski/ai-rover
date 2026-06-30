#!/usr/bin/env python3
"""Minimal, ROS-free hardware check for the Waveshare UGV Rover.

Sends a wheel-speed command straight to the ESP32 over serial, holds it for a
moment, then stops. Run this FIRST to prove the serial link and motors work
before bringing up ROS 2. It always sends a stop on exit, even on Ctrl-C.

Examples:
    # nudge both wheels forward at 0.2 m/s for 1.5 s, then stop
    python3 serial_smoketest.py --port /dev/serial0 --left 0.2 --right 0.2 --duration 1.5

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


def send(ser, left: float, right: float) -> None:
    cmd = {"T": 1, "L": round(left, 4), "R": round(right, 4)}
    ser.write((json.dumps(cmd) + "\n").encode())
    print("->", cmd)


def main() -> None:
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default="/dev/serial0")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--left", type=float, default=0.2)
    p.add_argument("--right", type=float, default=0.2)
    p.add_argument("--duration", type=float, default=1.5)
    args = p.parse_args()

    for v in (args.left, args.right):
        if not -0.5 <= v <= 0.5:
            sys.exit("left/right must be within the ESP32 limit of -0.5..+0.5 m/s")

    print(f"Opening {args.port} @ {args.baud} ...")
    ser = serial.Serial(args.port, args.baud, timeout=1.0)
    try:
        send(ser, args.left, args.right)
        time.sleep(args.duration)
    finally:
        send(ser, 0.0, 0.0)   # always stop, even on Ctrl-C / error
        ser.close()
        print("stopped, port closed.")


if __name__ == "__main__":
    main()
