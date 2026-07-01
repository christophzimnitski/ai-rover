#!/usr/bin/env python3
"""Inject a scripted /cmd_vel step sequence for latency testing. Runs INSIDE the container.

    !!! ROVER MUST BE IN THE AIR (wheels free) -- this spins the motors. !!!

    ssh rover 'cd ~/rover/docker && docker compose exec -T rover /entrypoint.sh \
        python3 - < ~/rover/scripts/inject_steps.py'

Publishes /cmd_vel directly (bypassing the deadman) at 20 Hz so the driver watchdog
stays fed, stepping through forward/reverse/spin phases with clean edges, then zeroes
and stops. Pair it with diag_latency.py running in another exec to measure the
command->motor latency. Total run ~18 s.
"""
import time

import rclpy
from geometry_msgs.msg import Twist

# (label, linear.x m/s, angular.z rad/s, seconds)
SEQ = [
    ("settle",     0.00,  0.0, 1.0),
    ("fwd 0.15",   0.15,  0.0, 2.0),
    ("stop",       0.00,  0.0, 1.5),
    ("fwd 0.30",   0.30,  0.0, 2.0),
    ("stop",       0.00,  0.0, 1.5),
    ("rev 0.20",  -0.20,  0.0, 2.0),
    ("stop",       0.00,  0.0, 1.5),
    ("spin +2.4",  0.00,  2.4, 2.0),
    ("stop",       0.00,  0.0, 1.5),
]
RATE = 20.0


def main():
    rclpy.init()
    node = rclpy.create_node("inject_steps")
    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    time.sleep(0.5)  # let discovery settle
    period = 1.0 / RATE
    try:
        for label, v, w, dur in SEQ:
            node.get_logger().info(f"phase: {label}")
            msg = Twist()
            msg.linear.x = float(v)
            msg.angular.z = float(w)
            end = time.time() + dur
            while time.time() < end:
                pub.publish(msg)
                time.sleep(period)
    finally:
        for _ in range(10):
            pub.publish(Twist())
            time.sleep(period)
        node.get_logger().info("done; /cmd_vel zeroed")
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
