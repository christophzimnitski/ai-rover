from setuptools import find_packages, setup

package_name = "ugv_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="Christoph Zimnitski",
    maintainer_email="145026135+christophzimnitski@users.noreply.github.com",
    description="ROS 2 base driver for the Waveshare UGV Rover.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "ugv_driver = ugv_driver.driver_node:main",
        ],
    },
)
