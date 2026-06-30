import os
from glob import glob

from setuptools import find_packages, setup

package_name = "ugv_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Christoph Zimnitski",
    maintainer_email="145026135+christophzimnitski@users.noreply.github.com",
    description="Launch files and parameters for the Waveshare UGV Rover.",
    license="MIT",
    entry_points={"console_scripts": []},
)
