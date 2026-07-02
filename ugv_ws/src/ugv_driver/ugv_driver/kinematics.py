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


class SpeedTrim:
    """Slow per-wheel integral trim: make the commanded speed physically true.

    WHY. The stock ESP32 firmware maps {"T":1,"L":x} OPEN-LOOP to motor PWM
    (x*512, saturating at 255) -- "m/s" in, throttle out. So the same command
    gives different real speeds on tile vs carpet, and at low commands the rover
    sticks (static friction) until the operator pushes harder, then lurches.
    The board DOES report real encoder speed back (quantized ~0.15 m/s at the
    50ms window), which is enough signal to close the loop up here.

    HOW. An integrator per wheel nudges the outgoing command until the (EMA-
    filtered) measured speed matches the commanded speed. Pure I-control on top
    of a feedforward passthrough: output = cmd + trim. Slow by design -- it
    corrects surface/stiction bias within ~a second; it is not a fast servo.
    The trim is clamped (anti-windup) and reset whenever the command is zero or
    flips sign, so 'stop' is always exactly stop and reversals don't inherit a
    stale boost. ROS-free and pure so it is unit-testable (tests/ off-site).
    """

    def __init__(self, ki: float = 1.2, trim_limit: float = 0.25,
                 meas_alpha: float = 0.35) -> None:
        self.ki = float(ki)                  # 1/s, integral gain
        self.limit = float(trim_limit)       # max |trim| added to a command
        self.alpha = float(meas_alpha)       # EMA weight for the noisy measurement
        self._trim = 0.0
        self._meas = 0.0
        self._sign = 0                       # sign of the last non-zero command

    def reset(self) -> None:
        self._trim = 0.0
        self._meas = 0.0
        self._sign = 0

    def update(self, cmd: float, measured: float, dt: float) -> float:
        """One control tick: return the trimmed command for wheel speed `cmd`.

        `measured` is the board-reported wheel speed (quantized is fine), `dt`
        the time since the previous tick in seconds.
        """
        if cmd == 0.0:
            self.reset()                     # stop stays stop, immediately
            return 0.0
        sign = 1 if cmd > 0.0 else -1
        if sign != self._sign:
            self._trim = 0.0                 # direction change: drop stale boost
            self._meas = 0.0
            self._sign = sign
        self._meas += self.alpha * (measured - self._meas)
        if 0.0 < dt < 0.5:                   # ignore silly gaps (paused stream)
            self._trim += self.ki * (cmd - self._meas) * dt
            self._trim = clamp(self._trim, self.limit)
        return cmd + self._trim
