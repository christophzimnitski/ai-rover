#!/usr/bin/env bash
# Source ROS 2 and (if built) the rover workspace, then run whatever was asked.
set -e

source /opt/ros/jazzy/setup.bash

if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

exec "$@"
