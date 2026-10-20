from glob import glob

from setuptools import setup

package_name = "arm_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Forward kinematics, inverse kinematics and keyboard typing for the LeKiwi arm",
    license="GPL-3.0-only",
    entry_points={
        "console_scripts": [
            # name you type after "ros2 run arm_control" = python file : function to start
            "forward_kinematics = arm_control.fk_node:main",
            "inverse_kinematics = arm_control.ik_node:main",
            "keyboard_typer = arm_control.keyboard_typer:main",
            "pool_typer = arm_control.pool_typer:main",
        ],
    },
)
