"""ugv_driver: ROS 2 node that turns /cmd_vel into ESP32 serial commands.

Subscribes:  /cmd_vel   (geometry_msgs/Twist)
Emits:       newline-delimited JSON wheel-speed commands over serial.

This is the *permanent* base layer of the rover stack. Teleop (now) and
autonomy / cat-following (later) both just publish a Twist to /cmd_vel; nothing
below this node has to change when behavior on top of it changes.
"""
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState

from ugv_msgs.msg import Telemetry

from ugv_driver.kinematics import twist_to_wheels
from ugv_driver.serial_link import SerialLink


def _f(data: dict, key: str) -> float:
    """Coerce a board-feedback field to float, defaulting missing/non-numeric to 0.0."""
    val = data.get(key)
    return float(val) if isinstance(val, (int, float)) else 0.0


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
        # Telemetry (read path): stream the ESP32's feedback and republish it.
        self.declare_parameter("telemetry", True)             # read + publish board feedback
        self.declare_parameter("telemetry_interval_ms", 100)  # ESP32 feedback cadence
        self.declare_parameter("telemetry_rate", 10.0)        # Hz, ROS publish rate

        port = self.get_parameter("serial_port").value
        baud = int(self.get_parameter("baud").value)
        require = bool(self.get_parameter("require_serial").value)

        self._track = float(self.get_parameter("track_width").value)
        self._max = float(self.get_parameter("max_wheel_speed").value)
        self._angular_gain = float(self.get_parameter("angular_gain").value)
        self._min = float(self.get_parameter("min_wheel_speed").value)
        self._timeout = float(self.get_parameter("cmd_timeout").value)

        self._link = SerialLink(port=port, baud=baud, require_serial=require,
                                logger=self.get_logger())
        if not self._link.connected:
            self.get_logger().warning(
                f"Serial not connected on {port}; running DRY (commands not sent).")

        self._last_cmd_time = self.get_clock().now()
        self._stopped = True

        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
        # Watchdog: halt the rover if /cmd_vel stops arriving (node crash, SSH drop, ...).
        self.create_timer(self._timeout / 2.0, self._watchdog)

        self._setup_telemetry()

        self.get_logger().info(
            f"ugv_driver up: port={port} track_width={self._track}m "
            f"max={self._max}m/s min={self._min}m/s timeout={self._timeout}s")

    def _setup_telemetry(self) -> None:
        """Read the ESP32 feedback stream and republish it as typed ROS topics.

        /rover/telemetry (ugv_msgs/Telemetry)       -- typed wheel speeds, roll/pitch,
                                                        voltage. The stable, swappable seam
                                                        the dashboard subscribes to.
        /rover/battery   (sensor_msgs/BatteryState) -- typed voltage for low-battery logic.
        """
        self._last_telem = None
        if not bool(self.get_parameter("telemetry").value):
            return
        if not self._link.connected:
            self.get_logger().warning("telemetry enabled but serial not connected; skipping.")
            return

        self._telem_pub = self.create_publisher(Telemetry, "rover/telemetry", 10)
        self._battery_pub = self.create_publisher(BatteryState, "rover/battery", 10)

        interval = int(self.get_parameter("telemetry_interval_ms").value)
        rate = float(self.get_parameter("telemetry_rate").value)
        started = self._link.start_telemetry(interval_ms=interval, on_data=self._on_telem)
        if not started:
            self.get_logger().warning("could not start ESP32 telemetry stream.")
            return
        # Republish on a ROS timer (decoupled from the serial cadence, and on the
        # executor thread so we never publish from the reader thread).
        self.create_timer(1.0 / max(rate, 1.0), self._publish_telemetry)
        self.get_logger().info(
            f"telemetry: streaming ESP32 feedback @ {interval}ms -> "
            f"/rover/telemetry + /rover/battery @ {rate}Hz")

    def _on_telem(self, data: dict) -> None:
        # Called from the serial reader thread: just stash the latest (atomic assign).
        self._last_telem = data

    def _publish_telemetry(self) -> None:
        data = self._last_telem
        if not data:
            return
        stamp = self.get_clock().now().to_msg()

        msg = Telemetry()
        msg.header.stamp = stamp
        msg.header.frame_id = "base_link"
        msg.left_speed = _f(data, "L")
        msg.right_speed = _f(data, "R")
        msg.roll = _f(data, "r")
        msg.pitch = _f(data, "p")
        msg.voltage = _f(data, "v")
        self._telem_pub.publish(msg)

        v = data.get("v")
        if isinstance(v, (int, float)):
            batt = BatteryState()
            batt.header.stamp = stamp
            batt.voltage = float(v)
            batt.percentage = math.nan
            batt.present = True
            batt.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
            self._battery_pub.publish(batt)

    def _on_cmd_vel(self, msg: Twist) -> None:
        # Differential-drive kinematics: Twist (m/s, rad/s) -> left/right wheel m/s.
        # angular_gain is a driver-side tuning knob applied BEFORE the pure-SI mix;
        # keep it at 1.0 to leave /cmd_vel a faithful physical contract (tune feel in
        # the teleop layer instead).
        w = msg.angular.z * self._angular_gain
        left, right = twist_to_wheels(
            msg.linear.x, w, self._track, self._max, self._min)
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
