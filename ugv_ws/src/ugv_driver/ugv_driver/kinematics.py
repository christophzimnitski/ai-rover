"""Differential-drive kinematics: Twist -> left/right wheel speeds.

Deliberately ROS-free and side-effect-free so it can be imported and unit-tested
without rclpy or hardware (see tests/test_kinematics.py). driver_node.py owns the
ROS wiring and calls into here; this module owns only the math.

Units are SI throughout (m/s, rad/s). This is the physical contract behind
`/cmd_vel`: a Twist in real units maps to wheel speeds in real units. Any "feel"
tuning (stick scaling, turn authority) belongs ABOVE this seam, in the teleop
layer -- not here.
"""
from __future__ import annotations


def clamp(value: float, limit: float) -> float:
    """Clamp `value` to the symmetric range [-limit, +limit]."""
    return max(-limit, min(limit, value))


def apply_min(value: float, floor: float) -> float:
    """Bump a non-zero wheel command up to `floor` to beat static friction.

    Zero stays zero (so 'stop' and a centred stick still stop). This is what makes
    low-speed turns actually move instead of buzzing against friction. Sign is
    preserved; `floor <= 0` disables it.
    """
    if floor > 0.0 and value != 0.0 and abs(value) < floor:
        return floor if value > 0.0 else -floor
    return value


def twist_to_wheels(linear_x: float, angular_z: float, track_width: float,
                    max_speed: float, min_speed: float) -> tuple[float, float]:
    """Map a differential-drive Twist to (left, right) wheel speeds in m/s.

    `linear_x` m/s and `angular_z` rad/s are the physical command (any tuning gain
    is applied by the caller before this point). `track_width` is the centre-to-centre
    wheel/tread separation in metres. Each wheel is clamped to +/-`max_speed`, then a
    non-zero result below `min_speed` is bumped up to it (stiction floor).
    """
    half = track_width / 2.0
    left = apply_min(clamp(linear_x - angular_z * half, max_speed), min_speed)
    right = apply_min(clamp(linear_x + angular_z * half, max_speed), min_speed)
    return left, right
