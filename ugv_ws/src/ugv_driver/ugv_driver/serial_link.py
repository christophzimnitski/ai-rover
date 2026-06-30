"""Thin wrapper around the rover's ESP32 serial link.

The ESP32 "General Driver Board" speaks newline-delimited JSON at 115200 baud.
Movement is one command type:

    {"T": 1, "L": <left_mps>, "R": <right_mps>}   # L,R in m/s, range -0.5..+0.5

This module knows *nothing* about ROS. It is deliberately small and importable
on its own so it can be unit-tested and reused by scripts/serial_smoketest.py.
"""
from __future__ import annotations

import json
import threading

try:
    import serial  # pyserial
except ImportError:  # allow import where pyserial is absent (e.g. lint/test on Mac)
    serial = None

# ESP32 JSON command type for closed-loop wheel speed control (m/s).
CMD_SPEED_CTRL = 1


class SerialLink:
    """Owns the serial port and serializes wheel-speed commands to the ESP32."""

    def __init__(self, port: str = "/dev/serial0", baud: int = 115200,
                 require_serial: bool = True) -> None:
        self._port = port
        self._baud = baud
        self._lock = threading.Lock()
        self._ser = None

        if serial is None:
            if require_serial:
                raise RuntimeError("pyserial is not installed (apt: python3-serial)")
            return

        try:
            self._ser = serial.Serial(port, baud, timeout=1.0)
        except Exception:
            if require_serial:
                raise
            # dry-run: keep going so the node can be exercised without hardware.
            self._ser = None

    @property
    def connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def _write(self, obj: dict) -> None:
        line = (json.dumps(obj) + "\n").encode("utf-8")
        with self._lock:
            if self._ser is not None:
                self._ser.write(line)

    def drive(self, left: float, right: float) -> None:
        """Set the left/right wheel target speed in m/s."""
        self._write({"T": CMD_SPEED_CTRL,
                     "L": round(float(left), 4),
                     "R": round(float(right), 4)})

    def stop(self) -> None:
        self.drive(0.0, 0.0)

    def close(self) -> None:
        try:
            self.stop()
        except Exception:
            pass
        with self._lock:
            if self._ser is not None:
                self._ser.close()
                self._ser = None
