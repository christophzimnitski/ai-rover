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
import time
from typing import Callable, Optional

try:
    import serial  # pyserial
except ImportError:  # allow import where pyserial is absent (e.g. lint/test on Mac)
    serial = None

# ESP32 JSON command type for closed-loop wheel speed control (m/s).
CMD_SPEED_CTRL = 1
# Telemetry stream control (Waveshare "General Driver Board"):
#   {"T":142,"cmd":<ms>}  set the feedback interval, then
#   {"T":131,"cmd":1|0}   turn the newline-delimited JSON feedback stream on|off.
# The board then emits {"T":1001,...} lines carrying wheel speeds, IMU and voltage.
CMD_FEEDBACK_INTERVAL = 142
CMD_FEEDBACK_FLOW = 131


class SerialLink:
    """Owns the serial port and serializes wheel-speed commands to the ESP32."""

    def __init__(self, port: str = "/dev/serial0", baud: int = 115200,
                 require_serial: bool = True) -> None:
        self._port = port
        self._baud = baud
        self._lock = threading.Lock()      # serializes WRITES only (reads are lock-free)
        self._ser = None

        # Background telemetry reader (see start_telemetry). Reads and writes share the
        # one fd but run in opposite directions, which is safe on a serial port.
        self._on_telem: Optional[Callable[[dict], None]] = None
        self._read_thread: Optional[threading.Thread] = None
        self._stop_read = threading.Event()
        self._last_telem: Optional[dict] = None

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

    # --- telemetry (read path) --------------------------------------------
    def start_telemetry(self, interval_ms: int = 100,
                        on_data: Optional[Callable[[dict], None]] = None) -> bool:
        """Enable the ESP32 feedback stream and read it on a background thread.

        The ESP32 emits newline-delimited JSON (e.g. {"T":1001,...} with wheel
        speeds, IMU and battery voltage). `on_data` is called with each parsed dict
        from the reader thread; the latest is also kept in `last_telemetry`. The read
        is lock-free and runs opposite the (locked) write path on the same port.
        """
        if self._ser is None:
            return False
        self._on_telem = on_data
        # Turn the stream on at the requested cadence.
        self._write({"T": CMD_FEEDBACK_INTERVAL, "cmd": int(interval_ms)})
        self._write({"T": CMD_FEEDBACK_FLOW, "cmd": 1})
        self._stop_read.clear()
        self._read_thread = threading.Thread(
            target=self._read_loop, name="esp32-telemetry", daemon=True)
        self._read_thread.start()
        return True

    @property
    def last_telemetry(self) -> Optional[dict]:
        return self._last_telem

    def _read_loop(self) -> None:
        while not self._stop_read.is_set():
            ser = self._ser
            if ser is None:
                break
            try:
                raw = ser.readline()          # blocks up to the port timeout (1.0s)
            except Exception:
                time.sleep(0.2)               # transient hiccup: back off and retry
                continue
            if not raw:
                continue
            try:
                data = json.loads(raw.decode("utf-8", "ignore"))
            except ValueError:
                continue                      # partial line / non-JSON: skip
            if isinstance(data, dict):
                self._last_telem = data
                if self._on_telem is not None:
                    try:
                        self._on_telem(data)
                    except Exception:
                        pass                  # never let a consumer kill the reader

    def close(self) -> None:
        # Stop the reader first so it isn't blocked on a port we're about to close.
        self._stop_read.set()
        if self._read_thread is not None:
            self._read_thread.join(timeout=1.5)
            self._read_thread = None
        try:
            if self._ser is not None:
                self._write({"T": CMD_FEEDBACK_FLOW, "cmd": 0})  # stop the stream
        except Exception:
            pass
        try:
            self.stop()
        except Exception:
            pass
        with self._lock:
            if self._ser is not None:
                self._ser.close()
                self._ser = None
