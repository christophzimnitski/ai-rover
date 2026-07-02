"""Unit tests for the differential-drive kinematics (ROS-free, hardware-free).

Covers the math that used to be verified only by driving the rover into furniture:
the Twist -> wheel mix, the symmetric clamp, and the stiction floor edge cases.
"""
import math

from ugv_driver.kinematics import SpeedTrim, apply_min, clamp, twist_to_wheels

TRACK = 0.175   # m, matches config/driver_params.yaml
MAX = 0.5       # m/s
NO_FLOOR = 0.0


# --- clamp ----------------------------------------------------------------
def test_clamp_within_range_is_unchanged():
    assert clamp(0.3, 0.5) == 0.3
    assert clamp(-0.3, 0.5) == -0.3


def test_clamp_saturates_both_signs():
    assert clamp(1.2, 0.5) == 0.5
    assert clamp(-1.2, 0.5) == -0.5


# --- apply_min (stiction floor) -------------------------------------------
def test_apply_min_zero_stays_zero():
    # 'stop' and a centred stick must remain a hard zero.
    assert apply_min(0.0, 0.10) == 0.0


def test_apply_min_bumps_small_nonzero_preserving_sign():
    assert apply_min(0.03, 0.10) == 0.10
    assert apply_min(-0.03, 0.10) == -0.10


def test_apply_min_leaves_values_at_or_above_floor():
    assert apply_min(0.10, 0.10) == 0.10
    assert apply_min(0.4, 0.10) == 0.4


def test_apply_min_disabled_when_floor_zero():
    assert apply_min(0.01, 0.0) == 0.01


# --- twist_to_wheels ------------------------------------------------------
def test_straight_forward_drives_both_wheels_equally():
    left, right = twist_to_wheels(0.3, 0.0, TRACK, MAX, NO_FLOOR)
    assert left == right == 0.3


def test_pure_rotation_is_antisymmetric():
    # Spin in place: wheels equal and opposite, magnitude = w * track/2.
    left, right = twist_to_wheels(0.0, 2.0, TRACK, MAX, NO_FLOOR)
    assert math.isclose(left, -right)
    assert math.isclose(right, 2.0 * TRACK / 2.0)


def test_positive_angular_turns_left_wheel_slower():
    # +angular_z -> right faster than left (turns toward -x side per REP-103 sign).
    left, right = twist_to_wheels(0.3, 1.0, TRACK, MAX, NO_FLOOR)
    assert right > left


def test_wheels_are_clamped_to_max():
    # Large linear + large angular would exceed the ESP32 limit; both clamp to max.
    left, right = twist_to_wheels(2.0, 10.0, TRACK, MAX, NO_FLOOR)
    assert left == MAX
    assert right == MAX


def test_stiction_floor_applied_after_mix():
    # A tiny forward command below the floor is bumped up (both wheels).
    left, right = twist_to_wheels(0.02, 0.0, TRACK, MAX, 0.10)
    assert left == right == 0.10


def test_stop_command_stays_stopped_with_floor_on():
    # Exact zero Twist must not be lifted to the floor.
    assert twist_to_wheels(0.0, 0.0, TRACK, MAX, 0.10) == (0.0, 0.0)


# --- SpeedTrim (Pi-side closed loop over the open-loop firmware) ------------
DT = 0.05   # 20Hz ticks, matching the telemetry/teleop cadence


def test_trim_zero_command_is_hard_zero():
    trim = SpeedTrim()
    trim.update(0.2, 0.0, DT)          # build some trim first
    assert trim.update(0.0, 0.15, DT) == 0.0


def test_trim_boosts_a_stalled_wheel():
    # Wheel commanded 0.2 but measured 0 (stiction): output must grow above cmd.
    trim = SpeedTrim(ki=1.2, trim_limit=0.25)
    out = 0.2
    for _ in range(20):                # one second of stall
        out = trim.update(0.2, 0.0, DT)
    assert out > 0.25                  # meaningful boost accumulated...
    assert out <= 0.2 + 0.25           # ...but never beyond cmd + limit


def test_trim_converges_to_passthrough_on_ideal_wheel():
    # Closed loop against a perfect actuator (measured follows the last output):
    # after the spin-up transient the trim must unwind to ~0 (output == command).
    trim = SpeedTrim()
    meas = 0.0
    for _ in range(100):               # 5s at 20Hz
        out = trim.update(0.3, meas, DT)
        meas = out                     # ideal wheel, one-tick delay
    assert math.isclose(out, 0.3, abs_tol=0.005)


def test_trim_breaks_stiction_and_holds_true_speed():
    # Sticky wheel: dead until the command reaches a 0.28 breakaway, then tracks
    # output minus 0.05 drag. The trim must break it loose and settle with the
    # MEASURED speed at the commanded 0.2 (output ~0.25 doing the compensating).
    trim = SpeedTrim()
    meas, broken = 0.0, False
    for _ in range(100):
        out = trim.update(0.2, meas, DT)
        broken = broken or out >= 0.28
        meas = max(0.0, out - 0.05) if broken else 0.0
    assert broken                      # stiction was broken by the boost
    assert math.isclose(meas, 0.2, abs_tol=0.01)


def test_trim_unwinds_after_stiction_breaks():
    # Stall builds boost; once the wheel overshoots, the boost must bleed off.
    trim = SpeedTrim(ki=1.2, trim_limit=0.25)
    for _ in range(20):
        trim.update(0.2, 0.0, DT)      # stalled: boost up
    boosted = trim.update(0.2, 0.0, DT)
    for _ in range(40):
        out = trim.update(0.2, 0.30, DT)   # now overshooting: unwind
    assert out < boosted
    assert out < 0.2 + 0.05


def test_trim_resets_on_direction_change():
    # Forward stall boost must not leak into a reverse command.
    trim = SpeedTrim()
    for _ in range(20):
        trim.update(0.2, 0.0, DT)
    first_reverse = trim.update(-0.2, 0.0, DT)
    assert abs(first_reverse - (-0.2)) < 0.02


def test_trim_ignores_silly_dt_gaps():
    # A paused stream (dt >= 0.5s) must not integrate a huge step.
    trim = SpeedTrim()
    out = trim.update(0.2, 0.0, 3.0)
    assert math.isclose(out, 0.2, abs_tol=1e-9)
