"""Camera bring-up: a USB UVC camera -> /camera/image_raw (THE SWAPPABLE SEAM).

Runs usb_cam in the `camera` namespace so it publishes the stable topics that the web
layer and dashboard subscribe to, independent of which camera is plugged in:

    /camera/image_raw             (sensor_msgs/Image)            -- raw frames (future CV)
    /camera/image_raw/compressed  (sensor_msgs/CompressedImage)  -- lighter transport
    /camera/camera_info           (sensor_msgs/CameraInfo)

To SWAP cameras, change only this file and config/camera.yaml:
  - another USB UVC cam  -> just point video_device at it (config/camera.yaml)
  - a Pi CSI camera      -> swap the node below to `camera_ros` (libcamera)
  - an IP / RTSP camera  -> a gstreamer bridge node publishing the same topic
The /camera/image_raw seam, web_video_server, and the frontend never change.

    ros2 launch ugv_bringup camera.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

# --- C950 image controls (applied via v4l2-ctl -- see fix_exposure below) -------------
# usb_cam does NOT reliably apply exposure to this camera: it leaves the cam in
# Aperture-Priority AUTO, which meters on the bright background and comes up WAY
# underexposed. v4l2-ctl drives the V4L2 device directly, so we pin a MANUAL exposure
# here instead. Tuned live against the rover's forward view (foreground sharp, highlights
# not blown). To retune, change these three; ranges on the C950 are exposure 10-500,
# gain 0-100, brightness 0-255.
_VIDEO = "/dev/video0"
_EXPOSURE = 250      # exposure_time_absolute (100µs units); larger = brighter + more blur
_GAIN = 60           # sensor gain; larger = brighter + noisier
_BRIGHTNESS = 128    # UVC brightness (the cam had drifted down to 50 = too dark)


def generate_launch_description() -> LaunchDescription:
    params = os.path.join(
        get_package_share_directory("ugv_bringup"), "config", "camera.yaml")

    # Force manual exposure a few seconds AFTER usb_cam grabs the device (so our values win
    # over whatever usb_cam sets at start-up), and re-apply a couple more times to cover a
    # slow start on the SD card. Control-name fallbacks (auto_exposure/exposure_auto,
    # exposure_time_absolute/exposure_absolute) keep this working across kernel versions.
    # White-balance/focus are left on the camera's own auto (they look correct).
    fix_exposure = TimerAction(period=1.0, actions=[
        ExecuteProcess(
            cmd=["bash", "-c",
                 "for i in 1 2 3; do "
                 f"v4l2-ctl -d {_VIDEO} --set-ctrl=auto_exposure=1 2>/dev/null || "
                 f"v4l2-ctl -d {_VIDEO} --set-ctrl=exposure_auto=1 2>/dev/null; "
                 f"v4l2-ctl -d {_VIDEO} "
                 f"--set-ctrl=exposure_time_absolute={_EXPOSURE},gain={_GAIN},brightness={_BRIGHTNESS} 2>/dev/null || "
                 f"v4l2-ctl -d {_VIDEO} "
                 f"--set-ctrl=exposure_absolute={_EXPOSURE},gain={_GAIN},brightness={_BRIGHTNESS} 2>/dev/null; "
                 "sleep 4; done"],
            output="screen"),
    ])

    return LaunchDescription([
        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            name="usb_cam",
            namespace="camera",      # -> /camera/image_raw, /camera/camera_info, ...
            output="screen",
            parameters=[params],
        ),
        fix_exposure,
    ])
