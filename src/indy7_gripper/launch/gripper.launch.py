from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("indy7_gripper")

    mode = LaunchConfiguration("mode").perform(context)
    robot_ip = LaunchConfiguration("robot_ip").perform(context)

    if mode not in ["sim", "real"]:
        raise RuntimeError("mode must be either 'sim' or 'real'")

    if mode == "sim":
        param_file = os.path.join(pkg_share, "config", "gripper_sim.yaml")
    else:
        param_file = os.path.join(pkg_share, "config", "gripper_real.yaml")

    node = Node(
        package="indy7_gripper",
        executable="gripper_node.py",
        name="indy7_gripper_node",
        output="screen",
        parameters=[
            param_file,
            {
                "mode": mode,
                "robot_ip": robot_ip,
            },
        ],
    )

    return [node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="sim",
                description="Gripper backend mode: sim or real",
            ),
            DeclareLaunchArgument(
                "robot_ip",
                default_value="166.104.234.72",
                description="Indy controller IP address for real mode",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )