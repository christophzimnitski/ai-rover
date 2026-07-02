"""Web transport layer for the dashboard (camera-agnostic, swappable panel).

Starts the two servers the browser talks to, and NOTHING rover-specific -- so this whole
layer can crash or be swapped without touching the driver or the camera:

    rosbridge_websocket (:9090)  JSON/WebSocket  -> roslibjs subscribes /rover/telemetry,
                                                    /rover/battery, /joy, ... and (later)
                                                    publishes /cmd_vel via twist_mux.
    rosapi              (        )                -> topic/type introspection for roslibjs.
    web_video_server    (:8080)  MJPEG over HTTP -> <img> src for the live camera view; it
                                                    serves ANY image topic, so swapping the
                                                    camera never touches it. Default view:
                                    http://<pi>:8080/stream_viewer?topic=/camera/image_raw

Run as its own compose service (see docker-compose.yml `webbridge`) so the base rover stack
stays up if the web layer dies.

    ros2 launch ugv_bringup webbridge.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        # ROS <-> browser JSON/WebSocket bridge.
        Node(
            package="rosbridge_server",
            executable="rosbridge_websocket",
            name="rosbridge_websocket",
            output="screen",
            parameters=[{"port": 9090}],
        ),
        # Topic/service/type introspection that roslibjs relies on.
        Node(
            package="rosapi",
            executable="rosapi_node",
            name="rosapi",
            output="screen",
        ),
        # Image topic -> MJPEG over HTTP. Binds all interfaces for LAN / Tailscale access.
        Node(
            package="web_video_server",
            executable="web_video_server",
            name="web_video_server",
            output="screen",
            parameters=[{"port": 8080, "address": "0.0.0.0"}],
        ),
    ])
