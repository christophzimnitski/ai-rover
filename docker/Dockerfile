# ROS 2 Jazzy (current LTS) runtime for the rover. Runs on Raspberry Pi OS via Docker,
# so we get ROS 2 without reflashing the Pi to Ubuntu.
#
# Pinned by digest for reproducible rebuilds -- the floating `jazzy-ros-base` tag moves,
# which would otherwise let a rebuild silently drift. Digest captured from Docker Hub on
# 2026-06-02. To refresh it later, run ON THE PI (which can verify it against the registry):
#     docker buildx imagetools inspect ros:jazzy-ros-base --format '{{.Manifest.Digest}}'
# and replace the sha256 below. If a build ever fails with "manifest unknown", the pin is
# just stale -- refresh it the same way (it is a manifest-list digest, so it is arch-safe).
FROM ros:jazzy-ros-base@sha256:6513503d0b10e919fbe8134981d4f9d19b5c1f9b045b87a9fe3b0b9e03e7c2a9

# Runtime deps:
#   python3-serial               -> the driver talks to the ESP32 over UART
#   teleop-twist-keyboard        -> WASD-style /cmd_vel publisher for first light
#   joy + teleop-twist-joy       -> gamepad: /dev/input/js0 -> /joy -> /cmd_vel
#   colcon + rosdep tooling      -> build the workspace inside the container
# Build deps for custom interfaces (ugv_msgs = ament_cmake + rosidl):
#   build-essential + cmake      -> compile the generated message typesupport
#   rosidl-default-generators    -> generate the Python/C bindings from msg/*.msg
# Webapp + camera stack (see the camera/webbridge services in docker-compose.yml):
#   usb-cam                      -> USB UVC camera -> /camera/image_raw (the swappable seam)
#   web-video-server             -> serve ANY image topic as MJPEG over HTTP :8080
#   rosbridge-suite              -> JSON/WebSocket ROS bridge :9090 for the roslibjs frontend
#   image-transport-plugins      -> compressed image transports (lighter camera streaming)
#   v4l-utils                    -> v4l2-ctl: inspect + live-tune camera controls (exposure, gain)
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-serial \
        ros-jazzy-teleop-twist-keyboard \
        ros-jazzy-joy \
        ros-jazzy-teleop-twist-joy \
        python3-colcon-common-extensions \
        build-essential \
        cmake \
        ros-jazzy-rosidl-default-generators \
        ros-jazzy-usb-cam \
        ros-jazzy-web-video-server \
        ros-jazzy-rosbridge-suite \
        ros-jazzy-image-transport-plugins \
        v4l-utils \
    && rm -rf /var/lib/apt/lists/*

# The colcon workspace is bind-mounted here at runtime (see docker-compose.yml),
# so the image stays small and code is live-editable from the Mac.
WORKDIR /ros2_ws

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
