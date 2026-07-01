"""Gamepad teleop layer: joy_node -> teleop_twist_joy -> /cmd_vel.

Launched on its own for bring-up/verification:

    ros2 launch ugv_bringup joystick.launch.py

or (normally) included by rover.launch.py so `docker compose up` brings up the
driver AND the gamepad together. Controller mapping lives in
config/teleop_joy.yaml -- see that file to retune or swap controllers.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    params = os.path.join(
        get_package_share_directory("ugv_bringup"), "config", "teleop_joy.yaml")

    return LaunchDescription([
        # joy_supervisor runs joy_node (which reads /dev/input/js0 and publishes /joy)
        # as a child and AUTO-RESTARTS it when the pad sleeps/wakes or the dongle
        # re-enumerates js0 -- otherwise joy_node clings to a dead fd and /joy dies
        # silently (there is no udevd in the container). It passes teleop_joy.yaml
        # straight through to joy_node, so the mapping/deadzone/autorepeat are unchanged.
        Node(
            package="ugv_bringup",
            executable="joy_supervisor",
            name="joy_supervisor",
            output="screen",
            parameters=[{"joy_params_file": params, "device_id": 0}],
        ),
        # Maps /joy (sticks/buttons) to geometry_msgs/Twist on /cmd_vel.
        Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            name="teleop_twist_joy_node",
            output="screen",
            parameters=[params],
        ),
    ])
