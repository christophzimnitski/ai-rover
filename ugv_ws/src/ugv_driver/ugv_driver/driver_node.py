"""ugv_driver: ROS 2 node that turns /cmd_vel into ESP32 serial commands.

Subscribes:  /cmd_vel   (geometry_msgs/Twist)
Emits:       newline-delimited JSON wheel-speed commands over serial.

This is the *permanent* base layer of the rover stack. Teleop (now) and
autonomy / cat-following (later) both just publish a Twist to /cmd_vel; nothing
below this node has to change when behavior on top of it changes.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from ugv_driver.serial_link import SerialLink


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _apply_min(value: float, floor: float) -> float:
    """Bump a non-zero wheel command up to `floor` to beat static friction.

    Zero stays zero (so 'stop' and a centred stick still stop). This is what makes
    low-speed turns actually move instead of buzzing against friction.
    """
    if floor > 0.0 and value != 0.0 and abs(value) < floor:
        return floor if value > 0.0 else -floor
    return value


class UgvDriver(Node):
    def __init__(self) -> None:
        super().__init__("ugv_driver")

        # --- parameters (override via config/driver_params.yaml) ---
        self.declare_parameter("serial_port", "/dev/serial0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("track_width", 0.13)      # m, left<->right wheel separation
        self.declare_parameter("max_wheel_speed", 0.5)   # m/s, ESP32 T:1 hard limit
        self.declare_parameter("angular_gain", 1.0)      # tuning knob for turn authority
        self.declare_parameter("min_wheel_speed", 0.0)   # m/s, stiction floor (0 = off)
        self.declare_parameter("cmd_timeout", 0.5)       # s, watchdog -> stop if cmd_vel goes quiet
        self.declare_parameter("require_serial", True)

        port = self.get_parameter("serial_port").value
        baud = int(self.get_parameter("baud").value)
        require = bool(self.get_parameter("require_serial").value)

        self._track = float(self.get_parameter("track_width").value)
        self._max = float(self.get_parameter("max_wheel_speed").value)
        self._angular_gain = float(self.get_parameter("angular_gain").value)
        self._min = float(self.get_parameter("min_wheel_speed").value)
        self._timeout = float(self.get_parameter("cmd_timeout").value)

        self._link = SerialLink(port=port, baud=baud, require_serial=require)
        if not self._link.connected:
            self.get_logger().warning(
                f"Serial not connected on {port}; running DRY (commands not sent).")

        self._last_cmd_time = self.get_clock().now()
        self._stopped = True

        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
        # Watchdog: halt the rover if /cmd_vel stops arriving (node crash, SSH drop, ...).
        self.create_timer(self._timeout / 2.0, self._watchdog)

        self.get_logger().info(
            f"ugv_driver up: port={port} track_width={self._track}m "
            f"max={self._max}m/s min={self._min}m/s timeout={self._timeout}s")

    def _on_cmd_vel(self, msg: Twist) -> None:
        # Differential-drive kinematics: Twist (m/s, rad/s) -> left/right wheel m/s.
        v = msg.linear.x
        w = msg.angular.z * self._angular_gain
        half = self._track / 2.0
        left = _apply_min(_clamp(v - w * half, self._max), self._min)
        right = _apply_min(_clamp(v + w * half, self._max), self._min)
        self._link.drive(left, right)
        self._last_cmd_time = self.get_clock().now()
        self._stopped = False

    def _watchdog(self) -> None:
        if self._stopped:
            return
        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if elapsed >= self._timeout:
            self._link.stop()
            self._stopped = True
            self.get_logger().debug("watchdog: cmd_vel quiet -> stop")

    def destroy_node(self) -> bool:
        self._link.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UgvDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
