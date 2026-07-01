#!/usr/bin/env python3
"""Latency / lag diagnostic for the rover control chain. Runs INSIDE the container.

    ssh rover 'cd ~/rover/docker && docker compose exec -T rover /entrypoint.sh \
        python3 - 30 < ~/rover/scripts/diag_latency.py'

Give it a duration (seconds). It subscribes to the whole chain --
  /joy (controller) -> /cmd_vel (teleop) -> [ugv_driver -> ESP32] -> /rover/telemetry
-- and after the window prints:
  * publish rate + inter-arrival jitter for each topic,
  * command->measured LAG from cross-correlation of commanded vs measured wheel speed,
  * step-response latency for clean 0->move edges (e.g. from inject_steps.py).

Receipt timestamps use time.perf_counter() (monotonic, sub-ms). The commanded wheel
speeds are reconstructed from /cmd_vel with the same track_width as the driver, so
they line up with the board's measured L/R regardless of the (unknown) L/R units --
cross-correlation is scale-invariant after de-meaning.
"""
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String

try:
    import numpy as np
except ImportError:
    np = None

TRACK_WIDTH = 0.175  # m -- must match driver_params.yaml to reconstruct cmd L/R


class Diag(Node):
    def __init__(self, duration: float):
        super().__init__("diag_latency")
        self._t0 = time.perf_counter()
        self._duration = duration
        self.joy, self.cmd, self.tel = [], [], []
        self.create_subscription(Joy, "/joy", self._on_joy, qos_profile_sensor_data)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.create_subscription(String, "/rover/telemetry", self._on_tel, 10)
        self.create_timer(1.0, self._tick)
        self.get_logger().info(f"recording for {duration:.0f}s ...")

    def _now(self):
        return time.perf_counter() - self._t0

    def _on_joy(self, m):
        dead = m.buttons[8] if len(m.buttons) > 8 else 0
        ly = m.axes[1] if len(m.axes) > 1 else 0.0
        rx = m.axes[2] if len(m.axes) > 2 else 0.0
        self.joy.append((self._now(), dead, ly, rx))

    def _on_cmd(self, m):
        v, w = m.linear.x, m.angular.z
        half = TRACK_WIDTH / 2.0
        self.cmd.append((self._now(), v, w, v - w * half, v + w * half))

    def _on_tel(self, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        self.tel.append((self._now(), d.get("L"), d.get("R"), d.get("v")))

    def _tick(self):
        t = self._now()
        c = self.cmd[-1] if self.cmd else None
        tel = self.tel[-1] if self.tel else None
        cs = f"cmd(v,w)={c[1]:+.2f},{c[2]:+.2f}" if c else "cmd=--"
        ms = f"meas(L,R)={tel[1]},{tel[2]}" if tel else "meas=--"
        self.get_logger().info(
            f"t={t:4.1f}s  joy={len(self.joy):4d} cmd={len(self.cmd):4d} "
            f"tel={len(self.tel):4d}  {cs}  {ms}")

    # ---- analysis --------------------------------------------------------
    def analyze(self):
        p = print
        p("\n================= DIAGNOSTIC SUMMARY =================")
        self._section("rates", self._rates)
        self._section("telemetry jitter", self._jitter)
        self._section("command -> measured lag (cross-correlation)", self._xcorr)
        self._section("step-response latency", self._steps)
        p("=====================================================")

    def _section(self, title, fn):
        print(f"\n--- {title} ---")
        try:
            fn()
        except Exception as e:  # never let one section kill the summary
            print(f"  (skipped: {e})")

    def _rates(self):
        for name, buf in (("/joy", self.joy), ("/cmd_vel", self.cmd),
                          ("/rover/telemetry", self.tel)):
            if len(buf) < 2:
                print(f"  {name:18s} n={len(buf)} (too few)")
                continue
            span = buf[-1][0] - buf[0][0]
            hz = (len(buf) - 1) / span if span > 0 else 0.0
            print(f"  {name:18s} n={len(buf):5d}  {hz:6.1f} Hz")

    def _jitter(self):
        if np is None or len(self.tel) < 3:
            print("  (need numpy + more telemetry)")
            return
        dt = np.diff([x[0] for x in self.tel]) * 1000.0  # ms
        print(f"  inter-arrival ms: mean={dt.mean():.1f} min={dt.min():.1f} "
              f"max={dt.max():.1f} std={dt.std():.1f}  (~{1000/dt.mean():.0f} Hz effective)")

    def _grid(self):
        tel = [x for x in self.tel if isinstance(x[1], (int, float))
               and isinstance(x[2], (int, float))]
        if np is None or len(self.cmd) < 20 or len(tel) < 20:
            raise RuntimeError("not enough numeric samples")
        tc = np.array([x[0] for x in self.cmd])
        cl = np.array([x[3] for x in self.cmd]); cr = np.array([x[4] for x in self.cmd])
        tt = np.array([x[0] for x in tel])
        mL = np.array([float(x[1]) for x in tel]); mR = np.array([float(x[2]) for x in tel])
        t0, t1 = max(tc[0], tt[0]), min(tc[-1], tt[-1])
        if t1 - t0 < 2.0:
            raise RuntimeError("cmd/telemetry overlap too short")
        g = np.arange(t0, t1, 0.005)
        return (g, np.interp(g, tc, cl), np.interp(g, tc, cr),
                np.interp(g, tt, mL), np.interp(g, tt, mR))

    def _xcorr(self):
        _, cl, cr, mL, mR = self._grid()
        for side, cmd, meas in (("L", cl, mL), ("R", cr, mR)):
            lag, peak = _best_lag(cmd, meas, 0.005)
            if lag is None:
                print(f"  {side}: no motion (flat signal) -- drive it to measure")
            else:
                print(f"  {side}: measured lags command by {lag*1000:+.0f} ms "
                      f"(corr peak {peak:.2f})")

    def _steps(self):
        if np is None:
            print("  (need numpy)"); return
        thr = 0.05
        edges, prev = [], 0.0
        for (t, _v, _w, cl, _cr) in self.cmd:
            if abs(prev) < thr <= abs(cl):
                edges.append(t)
            prev = cl
        if not edges:
            print("  no clean 0->move edges (run inject_steps.py or push the stick from center)")
            return
        tel = [(t, l) for (t, l, r, v) in self.tel if isinstance(l, (int, float))]
        lat = []
        for te in edges:
            base = [abs(l) for (t, l) in tel if te - 0.3 <= t < te]
            b = sum(base) / len(base) if base else 0.0
            win = [(t, abs(l)) for (t, l) in tel if te <= t <= te + 0.8]
            if not win:
                continue
            peak = max(v for _, v in win)
            if peak - b < 0.02:  # no discernible response
                continue
            target = b + 0.5 * (peak - b)
            hit = next((t for (t, v) in win if v >= target), None)
            if hit is not None:
                lat.append((hit - te) * 1000.0)
        if lat:
            a = np.array(lat)
            print(f"  {len(a)} steps: median={np.median(a):.0f} ms  "
                  f"mean={a.mean():.0f}  min={a.min():.0f}  max={a.max():.0f}")
            print("  (includes serial round-trip + firmware PID + physical motor spin-up)")
        else:
            print("  edges found but no measurable wheel response (check L/R units / motion)")


def _best_lag(cmd, meas, dt, max_lag_s=1.0):
    cmd = cmd - cmd.mean(); meas = meas - meas.mean()
    if cmd.std() < 1e-6 or meas.std() < 1e-6:
        return None, 0.0
    n = len(cmd)
    corr = np.correlate(meas, cmd, mode="full")   # index n-1 == lag 0
    maxk = int(max_lag_s / dt); mid = n - 1
    seg = corr[mid - maxk: mid + maxk + 1]
    lags = np.arange(-maxk, maxk + 1)
    k = lags[int(np.argmax(seg))]
    peak = seg.max() / (np.linalg.norm(cmd) * np.linalg.norm(meas) + 1e-9)
    return k * dt, float(peak)


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    rclpy.init()
    node = Diag(duration)
    try:
        while rclpy.ok() and node._now() < duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.analyze()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
