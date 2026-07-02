"""ugv_driver: ROS 2 node that turns /cmd_vel into ESP32 serial commands.

Subscribes:  /cmd_vel        (geometry_msgs/Twist)   primary drive source (gamepad, autonomy)
             /web_cmd_vel    (geometry_msgs/Twist)   secondary drive source (dashboard);
                                                     only applied while /cmd_vel is quiet
             /rover/estop    (std_msgs/Bool)         software e-stop latch (true = halt + ignore)
             /rover/lights   (ugv_msgs/Lights)       headlight PWM -> {"T":132}
             /rover/oled     (ugv_msgs/OledText)     OLED line -> {"T":3} / {"T":-3}
Publishes:   /rover/telemetry (ugv_msgs/Telemetry)   typed board feedback (the dashboard seam)
             /rover/battery   (sensor_msgs/BatteryState)
             /rover/link      (ugv_msgs/LinkStatus)  serial-link health @1Hz

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
from std_msgs.msg import Bool

from ugv_msgs.msg import Telemetry, Lights, OledText, LinkStatus

from ugv_driver.kinematics import twist_to_wheels, SpeedTrim
from ugv_driver.serial_link import (
    SerialLink, CMD_LED_CTRL, CMD_OLED_TEXT, CMD_OLED_DEFAULT, CMD_HEARTBEAT_SET)


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
        self.declare_parameter("max_wheel_speed", 0.5)   # m/s, {"T":1} saturates here (PWM 255)
        self.declare_parameter("angular_gain", 1.0)      # tuning knob for turn authority
        self.declare_parameter("min_wheel_speed", 0.0)   # m/s, stiction floor (0 = off)
        self.declare_parameter("cmd_timeout", 0.5)       # s, watchdog -> stop if cmd_vel goes quiet
        self.declare_parameter("require_serial", True)
        # Speed trim: Pi-side closed loop over the open-loop firmware (see kinematics.SpeedTrim).
        self.declare_parameter("wheel_trim", True)
        self.declare_parameter("trim_ki", 1.2)            # 1/s
        self.declare_parameter("trim_limit", 0.25)        # max |correction|
        # Secondary (web) drive source: applied only while /cmd_vel is quiet this long.
        self.declare_parameter("mux_window", 0.5)         # s
        # Board-side dead-man: firmware stops the motors itself if no command arrives
        # for this window ({"T":136}). Our 20Hz teleop stream keeps it fed while
        # driving; 0 disables (leaves the firmware default of 3000ms).
        self.declare_parameter("board_heartbeat_ms", 1000)
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
        self._mux_window = float(self.get_parameter("mux_window").value)
        self._heartbeat_ms = int(self.get_parameter("board_heartbeat_ms").value)

        self._link = SerialLink(port=port, baud=baud, require_serial=require,
                                logger=self.get_logger(),
                                on_link_up=self._on_link_up)
        if not self._link.connected:
            self.get_logger().warning(
                f"Serial not connected on {port}; running DRY (commands not sent).")

        self._last_cmd_time = self.get_clock().now()
        self._last_primary = None            # rclpy.Time of the last /cmd_vel msg
        self._stopped = True
        self._estop = False
        self._lights = (0, 0)                # last commanded (io4, io5), re-sent on link-up

        trim_on = bool(self.get_parameter("wheel_trim").value)
        ki = float(self.get_parameter("trim_ki").value)
        lim = float(self.get_parameter("trim_limit").value)
        self._trim_l = SpeedTrim(ki=ki, trim_limit=lim) if trim_on else None
        self._trim_r = SpeedTrim(ki=ki, trim_limit=lim) if trim_on else None
        self._last_trim_time = None

        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(Twist, "web_cmd_vel", self._on_web_cmd_vel, 10)
        self.create_subscription(Bool, "rover/estop", self._on_estop, 10)
        self.create_subscription(Lights, "rover/lights", self._on_lights, 10)
        self.create_subscription(OledText, "rover/oled", self._on_oled, 10)
        # Watchdog: halt the rover if drive commands stop arriving (node crash, SSH drop, ...).
        self.create_timer(self._timeout / 2.0, self._watchdog)

        self._link_pub = self.create_publisher(LinkStatus, "rover/link", 10)
        self.create_timer(1.0, self._publish_link)

        self._setup_telemetry()
        self._send_board_config()

        self.get_logger().info(
            f"ugv_driver up: port={port} track_width={self._track}m "
            f"max={self._max} min={self._min} timeout={self._timeout}s "
            f"trim={'on' if trim_on else 'off'} heartbeat={self._heartbeat_ms}ms")

    # --- board configuration -------------------------------------------------
    def _send_board_config(self) -> None:
        """Configuration frames the board forgets on reset (also sent on link-up)."""
        if not self._link.connected:
            return
        if self._heartbeat_ms > 0:
            self._link.send({"T": CMD_HEARTBEAT_SET, "cmd": self._heartbeat_ms})
        io4, io5 = self._lights
        if io4 or io5:                       # restore headlights after a board reset
            self._link.send({"T": CMD_LED_CTRL, "IO4": io4, "IO5": io5})

    def _on_link_up(self) -> None:
        # Called from the serial reader thread after a successful re-open; _write is
        # thread-safe so config frames may be sent from here directly.
        self._send_board_config()

    # --- telemetry (read path) ------------------------------------------------
    def _setup_telemetry(self) -> None:
        """Read the ESP32 feedback stream and republish it as typed ROS topics.

        /rover/telemetry (ugv_msgs/Telemetry)       -- typed wheel speeds, attitude,
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
        msg.yaw = _f(data, "y")              # 0.0 on firmware that doesn't send it
        msg.voltage = _f(data, "v")
        msg.temp = _f(data, "temp")          # 0.0 on firmware that doesn't send it
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

    def _publish_link(self) -> None:
        msg = LinkStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.connected = self._link.connected
        msg.estop = self._estop
        msg.telemetry_age = float(self._link.telemetry_age)
        msg.reconnects = int(self._link.reconnects)
        msg.rearms = int(self._link.rearms)
        self._link_pub.publish(msg)

    # --- drive (write path) ----------------------------------------------------
    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_primary = self.get_clock().now()
        self._apply_twist(msg)

    def _on_web_cmd_vel(self, msg: Twist) -> None:
        # Secondary source: the dashboard. The gamepad (or future autonomy) on
        # /cmd_vel always wins -- web commands are dropped while /cmd_vel is live.
        if self._last_primary is not None:
            quiet = (self.get_clock().now() - self._last_primary).nanoseconds / 1e9
            if quiet < self._mux_window:
                return
        self._apply_twist(msg)

    def _apply_twist(self, msg: Twist) -> None:
        if self._estop:
            return                            # latched: only /rover/estop false releases
        # Differential-drive kinematics: Twist (m/s, rad/s) -> left/right wheel m/s.
        # angular_gain is a driver-side tuning knob applied BEFORE the pure-SI mix;
        # keep it at 1.0 to leave /cmd_vel a faithful physical contract (tune feel in
        # the teleop layer instead).
        w = msg.angular.z * self._angular_gain
        left, right = twist_to_wheels(
            msg.linear.x, w, self._track, self._max, self._min)
        left, right = self._apply_trim(left, right)
        self._link.drive(left, right)
        self._last_cmd_time = self.get_clock().now()
        self._stopped = False

    def _apply_trim(self, left: float, right: float) -> tuple[float, float]:
        """Close the speed loop the firmware leaves open (see kinematics.SpeedTrim)."""
        if self._trim_l is None:
            return left, right
        telem = self._last_telem
        now = self.get_clock().now()
        dt = 0.0
        if self._last_trim_time is not None:
            dt = (now - self._last_trim_time).nanoseconds / 1e9
        self._last_trim_time = now
        if not telem:
            return left, right                # no measurement yet: pure feedforward
        meas_l = _f(telem, "L")
        meas_r = _f(telem, "R")
        left = self._trim_l.update(left, meas_l, dt)
        right = self._trim_r.update(right, meas_r, dt)
        # Trim must never push a wheel past the firmware's usable range.
        return (max(-self._max, min(self._max, left)),
                max(-self._max, min(self._max, right)))

    # --- accessories -----------------------------------------------------------
    def _on_estop(self, msg: Bool) -> None:
        if msg.data and not self._estop:
            self._estop = True
            self._halt()
            self.get_logger().warning("E-STOP engaged: drive commands ignored")
        elif not msg.data and self._estop:
            self._estop = False
            self.get_logger().info("e-stop released: drive commands accepted again")

    def _on_lights(self, msg: Lights) -> None:
        self._lights = (int(msg.io4), int(msg.io5))
        self._link.send({"T": CMD_LED_CTRL, "IO4": int(msg.io4), "IO5": int(msg.io5)})

    def _on_oled(self, msg: OledText) -> None:
        if int(msg.line) == 255:
            self._link.send({"T": CMD_OLED_DEFAULT})
        else:
            self._link.send({"T": CMD_OLED_TEXT,
                             "lineNum": int(msg.line), "Text": str(msg.text)})

    # --- safety ------------------------------------------------------------------
    def _halt(self) -> None:
        self._link.stop()
        self._stopped = True
        if self._trim_l is not None:
            self._trim_l.reset()
            self._trim_r.reset()

    def _watchdog(self) -> None:
        if self._stopped:
            return
        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if elapsed >= self._timeout:
            self._halt()
            self.get_logger().debug("watchdog: drive commands quiet -> stop")

    def destroy_node(self) -> bool:
        try:
            if self._lights != (0, 0):
                self._link.send({"T": CMD_LED_CTRL, "IO4": 0, "IO5": 0})
        except Exception:
            pass
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
