#!/usr/bin/env bash
# Source ROS 2 and (if built) the rover workspace, then run whatever was asked.
set -e

source /opt/ros/jazzy/setup.bash

if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
elif [[ "$*" == *ugv_* || "$*" == *"ros2 launch"* ]]; then
    # The workspace isn't built but the command needs it (the autostart launch or any
    # ugv_* run). install/ is gitignored and lives ONLY on the Pi's storage, so a fresh
    # Pi / new SD or SSD lands here -- fail LOUDLY with the fix instead of an opaque
    # crash-loop that just says "package 'ugv_bringup' not found". See scripts/bootstrap.sh.
    echo "==========================================================================" >&2
    echo "  ROVER WORKSPACE NOT BUILT: /ros2_ws/install/setup.bash is missing." >&2
    echo "  The ugv_bringup / ugv_driver / ugv_msgs packages are unavailable, so this" >&2
    echo "  command will fail. Build the workspace once, then it autostarts on boot:" >&2
    echo "      from the Mac:  scripts/build.sh        (or first-run: scripts/bootstrap.sh)" >&2
    echo "      on the Pi:     cd ~/rover/docker && \\" >&2
    echo "                     docker compose run --rm rover colcon build --symlink-install" >&2
    echo "==========================================================================" >&2
fi

exec "$@"
