from setuptools import setup
import os
from glob import glob

package_name = "indy7_task"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="leeseo",
    maintainer_email="you@example.com",
    description="Task-level control package for Indy7 pick/place/handover.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "task_node = indy7_task.task_node:main",
            "task_node_servo = indy7_task.task_node_servo:main",
            "task_node_preview = indy7_task.task_node_preview:main",
            "dcp3_monitor = indy7_task.dcp3_monitor:main",
        ],
    },
)
