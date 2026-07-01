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
        # Reads /dev/input/js0 and publishes sensor_msgs/Joy on /joy.
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
            parameters=[params],
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
