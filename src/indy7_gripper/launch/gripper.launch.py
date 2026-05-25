"""
Indy7 gripper launch file.

실행 예시:
  # Gazebo/RViz 시뮬레이션 gripper backend
  ros2 launch indy7_gripper gripper.launch.py mode:=sim

  # 실물 MPLM1630 gripper backend
  ros2 launch indy7_gripper gripper.launch.py mode:=real robot_ip:=166.104.214.96

터미널에서 gripper 서비스 테스트:
  ros2 service call /gripper/open std_srvs/srv/Trigger "{}"
  ros2 service call /gripper/close std_srvs/srv/Trigger "{}"
  ros2 service call /gripper/half_open std_srvs/srv/Trigger "{}"
  ros2 service call /gripper/state std_srvs/srv/Trigger "{}"

테스트 클라이언트:
  ros2 run indy7_gripper gripper_client_test.py open
  ros2 run indy7_gripper gripper_client_test.py close
"""

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
