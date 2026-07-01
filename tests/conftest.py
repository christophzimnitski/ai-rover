"""Pytest bootstrap for the standalone, off-site rover test suite.

These tests are a SEPARATE off-site helper: they run on the Mac / in CI and the
rover never imports, gates on, or ships them. Deleting this `tests/` dir (and
`.github/`) removes the checks with zero effect on the robot.

We test only the ROS-free logic -- `ugv_driver.kinematics` (pure math) and
`ugv_driver.serial_link` (talks to the board, degrades gracefully without
pyserial). Neither imports rclpy, so the suite needs only Python + pyserial and
never a ROS install. This shim puts the driver package source on sys.path so
pytest runs straight against the source tree, no `colcon`/`pip install` needed.
"""
import sys
from pathlib import Path

_DRIVER_SRC = Path(__file__).resolve().parent.parent / "ugv_ws" / "src" / "ugv_driver"
if str(_DRIVER_SRC) not in sys.path:
    sys.path.insert(0, str(_DRIVER_SRC))
