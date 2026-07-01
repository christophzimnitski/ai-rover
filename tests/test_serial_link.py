"""Tests for SerialLink -- the ROS-free wrapper around the ESP32 serial link.

Covers the deterministic command (write) path, dry-run safety, the telemetry
frame filter, and dead-port safety. No real hardware or serial port is touched: a
fake port records writes (or raises to simulate a dead fd).
"""
import json
import time

from ugv_driver.serial_link import FEEDBACK_TYPE, CMD_SPEED_CTRL, SerialLink


class FakeSerial:
    """Minimal stand-in for pyserial.Serial: records every line written."""

    def __init__(self):
        self.is_open = True
        self.writes: list[bytes] = []

    def write(self, data: bytes) -> None:
        self.writes.append(data)

    def close(self) -> None:
        self.is_open = False


class DeadSerial(FakeSerial):
    """A fake port whose fd has died: every write raises, as pyserial would."""

    def write(self, data: bytes) -> None:
        raise OSError("device disconnected")


def _dry_link() -> SerialLink:
    """A link that opened no real port (dry-run); safe to inject a FakeSerial into."""
    return SerialLink(port="/dev/nonexistent-test-port", baud=115200,
                      require_serial=False)


def _decode(fake: FakeSerial) -> list[dict]:
    return [json.loads(w.decode("utf-8")) for w in fake.writes]


# --- dry-run safety -------------------------------------------------------
def test_dry_run_reports_not_connected():
    link = _dry_link()
    assert link.connected is False


def test_dry_run_commands_are_safe_noops():
    # With no port, drive/stop must not raise (the node runs DRY without hardware).
    link = _dry_link()
    link.drive(0.2, 0.2)
    link.stop()
    link.close()


# --- command (write) path -------------------------------------------------
def test_drive_emits_speed_command_with_newline():
    link = _dry_link()
    fake = FakeSerial()
    link._ser = fake

    link.drive(0.25, -0.25)

    assert len(fake.writes) == 1
    assert fake.writes[0].endswith(b"\n")
    msg = _decode(fake)[0]
    assert msg == {"T": CMD_SPEED_CTRL, "L": 0.25, "R": -0.25}


def test_drive_rounds_to_four_decimals():
    link = _dry_link()
    fake = FakeSerial()
    link._ser = fake

    link.drive(0.123456, 0.654321)

    msg = _decode(fake)[0]
    assert msg["L"] == 0.1235
    assert msg["R"] == 0.6543


def test_stop_sends_zero_zero():
    link = _dry_link()
    fake = FakeSerial()
    link._ser = fake

    link.stop()

    assert _decode(fake)[0] == {"T": CMD_SPEED_CTRL, "L": 0.0, "R": 0.0}


# --- telemetry frame filter (only genuine {"T":1001} passes) --------------
def test_parse_accepts_feedback_frame():
    d = SerialLink._parse_feedback(b'{"T":1001,"L":0.1,"R":0.1,"r":0.0,"p":0.0,"v":12.2}\n')
    assert d is not None
    assert d["T"] == FEEDBACK_TYPE
    assert d["v"] == 12.2


def test_parse_rejects_command_echo():
    # A {"T":1} echo/ack is NOT telemetry and must not reach /rover/telemetry.
    assert SerialLink._parse_feedback(b'{"T":1,"L":0.2,"R":0.2}\n') is None


def test_parse_rejects_blank_and_garbage():
    assert SerialLink._parse_feedback(b"") is None
    assert SerialLink._parse_feedback(b"\n") is None
    assert SerialLink._parse_feedback(b"not json at all\n") is None


def test_parse_rejects_non_dict_json():
    assert SerialLink._parse_feedback(b"[1, 2, 3]\n") is None
    assert SerialLink._parse_feedback(b"42\n") is None


# --- dead-port safety (a control command must never crash the node) -------
def test_drive_swallows_write_failure_on_dead_port():
    link = _dry_link()
    link._ser = DeadSerial()
    link.drive(0.2, 0.2)   # write() raises internally; must NOT propagate
    link.stop()


# --- reconnect thrash guard ----------------------------------------------
def test_reopen_respects_thrash_guard():
    link = _dry_link()
    link._last_recover = time.monotonic()   # pretend we just recovered
    assert link._reopen() is False          # too soon -> no reopen attempt
