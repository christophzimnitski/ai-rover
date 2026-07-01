"""Thin wrapper around the rover's ESP32 serial link.

The ESP32 "General Driver Board" speaks newline-delimited JSON at 115200 baud.
Movement is one command type:

    {"T": 1, "L": <left_mps>, "R": <right_mps>}   # L,R in m/s, range -0.5..+0.5

This module knows *nothing* about ROS. It is deliberately small and importable
on its own so it can be unit-tested and reused by scripts/serial_smoketest.py.

SELF-HEALING. The link recovers on its own, the same way joy_supervisor keeps the
gamepad alive. The telemetry reader watches two failure signals:
  1. the port erroring (readline/write raising) -> close and re-open the fd, and
  2. the feedback stream going stale (no {"T":1001} frames) while the fd is fine
     -> the ESP32 reset/browned-out and forgot it was streaming, so re-arm it.
Both are rate-limited by min_reconnect_interval so a flapping link can't thrash.
"""
from __future__ import annotations

import json
import logging
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
# The board's telemetry frame type. Everything else on the wire (command echoes,
# other T-codes) is NOT telemetry and must not reach /rover/telemetry.
FEEDBACK_TYPE = 1001


class SerialLink:
    """Owns the serial port and serializes wheel-speed commands to the ESP32."""

    def __init__(self, port: str = "/dev/serial0", baud: int = 115200,
                 require_serial: bool = True, logger: object = None,
                 stale_timeout: float = 2.0,
                 min_reconnect_interval: float = 3.0) -> None:
        self._port = port
        self._baud = baud
        self._lock = threading.Lock()      # serializes WRITES + reopen (reads are lock-free)
        self._ser = None
        # Duck-typed logger: a ROS node logger or a stdlib Logger both expose
        # .info/.warning, and either way the text lands in `docker compose logs`.
        self._log = logger or logging.getLogger(__name__)

        # Background telemetry reader (see start_telemetry). Reads and writes share the
        # one fd but run in opposite directions, which is safe on a serial port.
        self._on_telem: Optional[Callable[[dict], None]] = None
        self._read_thread: Optional[threading.Thread] = None
        self._stop_read = threading.Event()
        self._last_telem: Optional[dict] = None

        # Recovery state (all touched only from the reader thread + start_telemetry).
        self._interval_ms: Optional[int] = None   # last requested feedback cadence
        self._stale_timeout = float(stale_timeout)
        self._min_reconnect_interval = float(min_reconnect_interval)
        self._seen_telem = False
        self._last_rx = time.monotonic()
        self._last_recover = 0.0                  # thrash guard for reopen + re-arm

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
            ser = self._ser
            if ser is None:
                return                        # port down: drop; the reader will reopen
            try:
                ser.write(line)
            except Exception:
                # Port died under us. Don't crash the caller (a control command must
                # never take down the node); the reader thread handles the reopen.
                pass

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
        speeds, IMU and battery voltage). `on_data` is called with each parsed
        {"T":1001} dict from the reader thread; the latest is also kept in
        `last_telemetry`. The read is lock-free and runs opposite the (locked) write
        path on the same port. The reader also self-heals the link (see module docstring).
        """
        if self._ser is None:
            return False
        self._on_telem = on_data
        self._interval_ms = int(interval_ms)
        self._arm_stream()                    # turn the stream on at the requested cadence
        self._stop_read.clear()
        self._last_rx = time.monotonic()
        self._read_thread = threading.Thread(
            target=self._read_loop, name="esp32-telemetry", daemon=True)
        self._read_thread.start()
        return True

    def _arm_stream(self) -> None:
        """(Re)tell the ESP32 to stream {"T":1001} feedback at the stored cadence.

        Sent at start-up and again after any recovery -- a reset board forgets it was
        streaming. Safe to call any time; must NOT be called while holding self._lock
        (each _write takes the lock).
        """
        if self._interval_ms is None:
            return
        self._write({"T": CMD_FEEDBACK_INTERVAL, "cmd": int(self._interval_ms)})
        self._write({"T": CMD_FEEDBACK_FLOW, "cmd": 1})

    @property
    def last_telemetry(self) -> Optional[dict]:
        return self._last_telem

    @staticmethod
    def _parse_feedback(raw: bytes) -> Optional[dict]:
        """Decode one serial line to a telemetry dict, or None if it isn't one.

        Only genuine {"T":1001} frames pass; blank lines, partial/non-JSON, non-dicts
        and other T-codes (command echoes, acks) are dropped. Pure and side-effect-free
        so it can be unit-tested without a port.
        """
        if not raw:
            return None
        try:
            data = json.loads(raw.decode("utf-8", "ignore"))
        except ValueError:
            return None                       # partial line / non-JSON: skip
        if isinstance(data, dict) and data.get("T") == FEEDBACK_TYPE:
            return data
        return None

    def _reopen(self) -> bool:
        """Close and re-open the serial fd. Rate-limited; re-arms the stream on success."""
        now = time.monotonic()
        if now - self._last_recover < self._min_reconnect_interval:
            return False
        self._last_recover = now
        if serial is None:
            return False
        with self._lock:
            try:
                if self._ser is not None:
                    self._ser.close()
            except Exception:
                pass
            try:
                self._ser = serial.Serial(self._port, self._baud, timeout=1.0)
                ok = True
            except Exception:
                self._ser = None
                ok = False
        if ok:
            self._arm_stream()                # arm OUTSIDE the lock (each _write locks)
            self._log.info("serial link recovered: re-opened %s" % self._port)
        else:
            self._log.warning("serial link down: re-open of %s failed" % self._port)
        return ok

    def _read_loop(self) -> None:
        while not self._stop_read.is_set():
            ser = self._ser
            if ser is None:
                if not self._reopen():        # port down (e.g. write failed): try to restore
                    time.sleep(0.2)
                continue
            try:
                raw = ser.readline()          # blocks up to the port timeout (1.0s)
            except Exception:
                self._log.warning("serial read error on %s; reconnecting" % self._port)
                self._reopen()                # fd errored -> close + re-open
                continue
            data = self._parse_feedback(raw)
            if data is not None:
                self._last_telem = data
                self._last_rx = time.monotonic()
                self._seen_telem = True
                if self._on_telem is not None:
                    try:
                        self._on_telem(data)
                    except Exception:
                        pass                  # never let a consumer kill the reader
            elif self._seen_telem and (time.monotonic() - self._last_rx) > self._stale_timeout:
                # Stream went quiet with a healthy fd: the board reset/browned-out and
                # forgot it was streaming. Re-arm it (rate-limited by min_reconnect_interval).
                if time.monotonic() - self._last_recover >= self._min_reconnect_interval:
                    self._last_recover = time.monotonic()
                    self._log.warning(
                        "telemetry stale for >%.1fs; re-arming ESP32 stream"
                        % self._stale_timeout)
                    self._arm_stream()

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
