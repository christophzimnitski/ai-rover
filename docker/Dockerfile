# ROS 2 Jazzy (current LTS) runtime for the rover. Runs on Raspberry Pi OS via Docker,
# so we get ROS 2 without reflashing the Pi to Ubuntu.
FROM ros:jazzy-ros-base

# Runtime deps:
#   python3-serial               -> the driver talks to the ESP32 over UART
#   teleop-twist-keyboard        -> WASD-style /cmd_vel publisher for first light
#   joy + teleop-twist-joy       -> gamepad: /dev/input/js0 -> /joy -> /cmd_vel
#   colcon + rosdep tooling      -> build the workspace inside the container
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-serial \
        ros-jazzy-teleop-twist-keyboard \
        ros-jazzy-joy \
        ros-jazzy-teleop-twist-joy \
        python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# The colcon workspace is bind-mounted here at runtime (see docker-compose.yml),
# so the image stays small and code is live-editable from the Mac.
WORKDIR /ros2_ws

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
