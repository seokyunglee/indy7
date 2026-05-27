"""
Indy7 MoveIt Gazebo launch file.

실행 예시:
  # Gazebo/RViz 시뮬레이션 + MoveIt
  ros2 launch indy7_moveit indy_moveit_gazebo.launch.py gripper_model:=sim


  # 실험 환경 world까지 같이 실행
  ros2 launch indy7_moveit indy_moveit_gazebo.launch.py gazebo_env:=pick_place gripper_model:=sim
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    
    # description_package = FindPackageShare('indy7_description')
    gazebo_package = FindPackageShare('indy7_gazebo')
    moveit_config_package = FindPackageShare('indy7_moveit')

    # Initialize Arguments
    name = LaunchConfiguration("name")
    indy_type = LaunchConfiguration("indy_type")
    indy_eye = LaunchConfiguration("indy_eye")
    servo_mode = LaunchConfiguration("servo_mode")
    prefix = LaunchConfiguration("prefix")
    gazebo_env = LaunchConfiguration("gazebo_env")
    gripper_model = LaunchConfiguration("gripper_model")

    indy_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [gazebo_package, "/launch", "/indy_gazebo.launch.py"]
        ),
        launch_arguments={
            "name": name,
            "indy_type": indy_type,
            "indy_eye": indy_eye,
            "prefix": prefix,
            "launch_rviz": "false",
            "gazebo_env": gazebo_env,
        }.items(),
    )

    indy_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [moveit_config_package, "/launch", "/moveit.launch.py"]
        ),
        launch_arguments={
            "name": name,
            "indy_type": indy_type,
            "indy_eye": indy_eye,
            "servo_mode": servo_mode,
            "prefix": prefix,
            "use_sim_time": "true",
            "gripper_model": gripper_model,
            "launch_rviz_moveit": "true", # if name == "launch_rviz" => spawn 2 rviz
        }.items(),
    )

    nodes_to_launch = [
        indy_gazebo_launch,
        indy_moveit_launch,
    ]

    return nodes_to_launch


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "name",
            default_value="indy"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "indy_type",
            default_value="indy7",
            description="Type of Indy robot.",
            choices=["indy7", "indy7_v2", "indy7_v3", "indy12", "indy12_v2", "indyrp2", "indyrp2_v2", "icon7l", "icon3", "nuri3s", "nuri4s", "nuri7c", "nuri20c", "opti5"]
        )
    )
 
    declared_arguments.append(
        DeclareLaunchArgument(
            "indy_eye",
            default_value="false",
            description="Work with Indy Eye",
        )
    )
 
    declared_arguments.append(
        DeclareLaunchArgument(
            "servo_mode",
            default_value="false",
            description="Servoing mode",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for multi-robot setup. \
            If changed than also joint names in the controllers configuration have to be updated."
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "gazebo_env",
            default_value="",
            description=(
                "Gazebo environment name. Empty/default uses the normal "
                "Gazebo world; pick_place loads indy7_gazebo/worlds/pick_place.world."
            ),
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_model",
            default_value="sim",
            choices=["none", "sim", "real_collision"],
            description="none: no gripper, sim: joint-controlled gripper, real_collision: fixed collision-only gripper.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
