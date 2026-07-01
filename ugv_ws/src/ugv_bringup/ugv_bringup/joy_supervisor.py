"""joy_supervisor: keep joy_node alive across pad sleep/wake and dongle re-enumeration.

THE PROBLEM. The GameSir dongle re-enumerates /dev/input/js0 (a new inode) whenever
the pad sleeps/wakes or disconnects/reconnects. joy_node opened js0 once at startup
and then clings to a now-dead fd -- /joy silently stops and the rover goes
unresponsive until someone restarts the container by hand. There is no udevd in the
container, so SDL's own hotplug can't recover the device.

THE FIX. Run joy_node as a CHILD of this node and watch two signals:
  1. the js device inode changing  -> disconnect/reconnect (re-enumeration), and
  2. /joy going stale while the device is present -> sleep/wake (dead fd, same inode;
     healthy joy_node republishes at autorepeat_rate, so silence == a dead fd).
On either, bounce ONLY joy_node. The base driver and teleop keep running; the fresh
joy_node opens the current js0 and control returns automatically within a second or two.

Safety is unaffected: ugv_driver's cmd_timeout already stops the wheels the moment
/cmd_vel stops arriving. This node restores control; it never sends motion.

teleop_joy.yaml is passed straight through to joy_node, so the mapping, deadzone and
autorepeat_rate you tuned are exactly as before -- this node only manages its lifecycle.
"""
from __future__ import annotations

import os
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy


class JoySupervisor(Node):
    def __init__(self) -> None:
        super().__init__("joy_supervisor")

        self.declare_parameter("device_id", 0)            # 0 = /dev/input/js0
        self.declare_parameter("joy_params_file", "")     # passed to joy_node --params-file
        self.declare_parameter("check_period", 1.0)       # s, how often to poll
        self.declare_parameter("stale_timeout", 2.0)      # s, /joy silence -> dead fd
        self.declare_parameter("min_restart_interval", 3.0)  # s, thrash guard

        self._device_id = int(self.get_parameter("device_id").value)
        self._params_file = str(self.get_parameter("joy_params_file").value)
        self._stale = float(self.get_parameter("stale_timeout").value)
        self._min_interval = float(self.get_parameter("min_restart_interval").value)
        period = float(self.get_parameter("check_period").value)

        self._dev = f"/dev/input/js{self._device_id}"
        self._child: subprocess.Popen | None = None
        # Inode the running joy_node is bound to, captured when it was spawned
        # (None if the pad was absent then). A live inode that differs from this means
        # the device changed under joy_node -> it needs a restart to reopen.
        self._bound_inode: int | None = None
        self._seen_joy = False
        self._last_joy = time.monotonic()
        self._last_restart = 0.0
        self._waiting_logged = False

        # Monitor /joy liveness. best_effort (sensor QoS) is compatible with joy_node's
        # publisher whether it offers reliable or best_effort.
        self.create_subscription(Joy, "joy", self._on_joy, qos_profile_sensor_data)

        self._start_joy_node()
        self.create_timer(period, self._tick)
        self.get_logger().info(
            f"joy_supervisor watching {self._dev} (stale_timeout={self._stale}s); "
            "auto-restarts joy_node on pad sleep/wake or reconnect")

    # --- joy_node child lifecycle -----------------------------------------
    def _start_joy_node(self) -> None:
        cmd = ["ros2", "run", "joy", "joy_node"]
        if self._params_file:
            cmd += ["--ros-args", "--params-file", self._params_file]
        # start_new_session=True -> own process group so we can signal the whole
        # group (the `ros2 run` wrapper AND joy_node) on restart.
        self._child = subprocess.Popen(cmd, start_new_session=True)
        # Bind to whatever device exists right now -- that's the inode joy_node will
        # open (or None if the pad is absent, in which case any future device differs).
        self._bound_inode = self._inode()
        self._seen_joy = False
        self._last_joy = time.monotonic()
        self._last_restart = time.monotonic()
        self.get_logger().info(f"started joy_node (pid {self._child.pid})")

    def _stop_joy_node(self) -> None:
        child = self._child
        self._child = None
        if child is None or child.poll() is not None:
            return
        try:
            pgid = os.getpgid(child.pid)
        except ProcessLookupError:
            return
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                return
            try:
                child.wait(timeout=2.0)
                return
            except subprocess.TimeoutExpired:
                continue

    def _restart(self, reason: str) -> None:
        if time.monotonic() - self._last_restart < self._min_interval:
            return  # thrash guard: don't restart faster than min_restart_interval
        self.get_logger().warning(f"restarting joy_node: {reason}")
        self._stop_joy_node()
        self._start_joy_node()

    # --- monitoring -------------------------------------------------------
    def _on_joy(self, _msg: Joy) -> None:
        self._seen_joy = True
        self._last_joy = time.monotonic()

    def _inode(self) -> int | None:
        try:
            return os.stat(self._dev).st_ino
        except OSError:
            return None

    def _tick(self) -> None:
        alive = self._child is not None and self._child.poll() is None
        inode = self._inode()
        present = inode is not None
        now = time.monotonic()

        if not alive:
            # joy_node exited on its own; bring it back once the device is here.
            if present:
                self._restart("joy_node process exited")
            return

        if not present:
            # Pad gone -- nothing to open. The driver's cmd_timeout already halted us.
            # Keep _bound_inode as-is so a return with a NEW inode is detected below.
            if not self._waiting_logged:
                self.get_logger().info("js device gone; waiting for the pad to return")
                self._waiting_logged = True
            return
        self._waiting_logged = False

        if inode != self._bound_inode:
            # Device changed under the running joy_node: it started without a pad and one
            # appeared, or the dongle re-enumerated js0 on reconnect. Reopen via restart.
            self._restart(f"js device changed (bound {self._bound_inode} -> {inode})")
        elif self._seen_joy and (now - self._last_joy) > self._stale:
            # Same inode but /joy went silent: dead fd from a sleep/wake. Reopen.
            self._restart(f"/joy stale for {now - self._last_joy:.1f}s (dead fd)")

    def destroy_node(self) -> bool:
        self._stop_joy_node()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JoySupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
