"""Thin wrapper around the rover's ESP32 serial link.

The ESP32 "General Driver Board" speaks newline-delimited JSON at 115200 baud.
Movement is one command type:

    {"T": 1, "L": <left>, "R": <right>}   # +-0.5; the stock firmware maps this
                                          # OPEN-LOOP to PWM (value*512, sat 255),
                                          # so it is a throttle fraction, not m/s.

This module knows *nothing* about ROS. It is deliberately small and importable
on its own so it can be unit-tested and reused by scripts/serial_smoketest.py.

SELF-HEALING. The link recovers on its own, the same way joy_supervisor keeps the
gamepad alive. The telemetry reader watches two failure signals:
  1. the port erroring (readline/write raising) -> close and re-open the fd, and
  2. the feedback stream going stale (no {"T":1001} frames) while the fd is fine
     -> first RE-ARM the stream (a reset board forgot it was streaming); if it is
     STILL stale one stale_timeout later, ESCALATE to a full fd re-open (observed
     in the field: after ~3h a wedged UART recovered only after close+reopen).
Both are rate-limited by min_reconnect_interval so a flapping link can't thrash.
on_link_up fires after every successful (re)open so the owner can re-send any
board configuration (heartbeat window, lights state, ...).
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

# ESP32 JSON command types (see docs/FIRMWARE.md for the full T-code protocol).
CMD_SPEED_CTRL = 1          # {"T":1,"L":x,"R":x}  open-loop wheel throttle
CMD_OLED_TEXT = 3           # {"T":3,"lineNum":n,"Text":s}
CMD_OLED_DEFAULT = -3       # {"T":-3} restore default OLED screen
CMD_GET_IMU = 126           # {"T":126} -> one {"T":1002} raw-IMU reply
CMD_FEEDBACK_ONCE = 130     # {"T":130} -> one {"T":1001} frame
CMD_FEEDBACK_FLOW = 131     # {"T":131,"cmd":1|0} feedback stream on|off
CMD_LED_CTRL = 132          # {"T":132,"IO4":0-255,"IO5":0-255} headlights
CMD_HEARTBEAT_SET = 136     # {"T":136,"cmd":<ms>} board-side dead-man window
CMD_FEEDBACK_INTERVAL = 142  # {"T":142,"cmd":<ms>} feedback cadence
# The board's telemetry frame type. Everything else on the wire (command echoes,
# other T-codes) is NOT telemetry and must not reach /rover/telemetry.
FEEDBACK_TYPE = 1001


class SerialLink:
    """Owns the serial port and serializes JSON commands to the ESP32."""

    def __init__(self, port: str = "/dev/serial0", baud: int = 115200,
                 require_serial: bool = True, logger: object = None,
                 stale_timeout: float = 2.0,
                 min_reconnect_interval: float = 3.0,
                 on_link_up: Optional[Callable[[], None]] = None) -> None:
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
        self._on_link_up = on_link_up
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
        self._rearmed_at = 0.0                    # when the stale path last re-armed
        # Health counters for /rover/link (monotonic, never reset).
        self.reconnects = 0
        self.rearms = 0

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

    @property
    def telemetry_age(self) -> float:
        """Seconds since the last {"T":1001} frame; -1.0 if none seen yet."""
        if not self._seen_telem:
            return -1.0
        return time.monotonic() - self._last_rx

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

    def send(self, obj: dict) -> None:
        """Send an arbitrary JSON command frame (lights, OLED, heartbeat, ...)."""
        self._write(obj)

    def drive(self, left: float, right: float) -> None:
        """Set the left/right wheel command (open-loop throttle, +-0.5 usable)."""
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
            self.reconnects += 1
            self._arm_stream()                # arm OUTSIDE the lock (each _write locks)
            if self._on_link_up is not None:
                try:
                    self._on_link_up()        # let the owner re-send board config
                except Exception:
                    pass
            self._log.info("serial link recovered: re-opened %s" % self._port)
        else:
            self._log.warning("serial link down: re-open of %s failed" % self._port)
        return ok

    def _handle_stale(self, now: float) -> None:
        """Stream quiet with a healthy-looking fd. Recover in two stages.

        Stage 1: re-arm the stream (a reset/browned-out board forgot it was
        streaming -- the cheap fix). Stage 2: if a re-arm was already tried and the
        stream is STILL quiet one stale_timeout later, the fd itself is wedged even
        though reads aren't raising -- close and re-open it, then re-arm (the fix
        that field-recovered the 3h-uptime hang). Both rate-limited.
        """
        if now - self._last_recover < self._min_reconnect_interval:
            return
        if self._rearmed_at <= self._last_rx:
            # First sign of staleness since data last flowed: try the cheap re-arm.
            self._last_recover = now
            self._rearmed_at = now
            self.rearms += 1
            self._log.warning(
                "telemetry stale for >%.1fs; re-arming ESP32 stream" % self._stale_timeout)
            self._arm_stream()
        else:
            # Re-arm didn't bring data back: the fd is wedged. Full re-open.
            self._log.warning(
                "telemetry still stale after re-arm; re-opening %s" % self._port)
            self._reopen()

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
                self._handle_stale(time.monotonic())

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
