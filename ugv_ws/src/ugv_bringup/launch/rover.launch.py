"""Bring up the rover base driver with parameters from config/driver_params.yaml.

    ros2 launch ugv_bringup rover.launch.py

Drive it by publishing geometry_msgs/Twist on /cmd_vel (e.g. teleop_twist_keyboard).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    params = os.path.join(
        get_package_share_directory("ugv_bringup"), "config", "driver_params.yaml")

    return LaunchDescription([
        Node(
            package="ugv_driver",
            executable="ugv_driver",
            name="ugv_driver",
            output="screen",
            parameters=[params],
        ),
    ])
