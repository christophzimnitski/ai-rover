"""Bring up the rover base driver AND the gamepad teleop layer.

    ros2 launch ugv_bringup rover.launch.py

Starts ugv_driver (config/driver_params.yaml) plus the joystick stack from
joystick.launch.py (joy_node + teleop_twist_joy). Drive it with the gamepad, or
publish geometry_msgs/Twist on /cmd_vel directly (e.g. teleop_twist_keyboard).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("ugv_bringup")
    params = os.path.join(share, "config", "driver_params.yaml")
    joystick_launch = os.path.join(share, "launch", "joystick.launch.py")

    return LaunchDescription([
        Node(
            package="ugv_driver",
            executable="ugv_driver",
            name="ugv_driver",
            output="screen",
            parameters=[params],
        ),
        # Gamepad -> /cmd_vel. Remove this include (or launch joystick.launch.py
        # separately) if you want the driver on its own.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(joystick_launch)),
    ])
