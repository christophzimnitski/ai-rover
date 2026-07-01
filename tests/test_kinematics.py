"""Unit tests for the differential-drive kinematics (ROS-free, hardware-free).

Covers the math that used to be verified only by driving the rover into furniture:
the Twist -> wheel mix, the symmetric clamp, and the stiction floor edge cases.
"""
import math

from ugv_driver.kinematics import apply_min, clamp, twist_to_wheels

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
