"""
Indy7 Gazebo launch file.

실행 예시:
  # 기본 Gazebo 환경 + Indy7 + 그리퍼
  ros2 launch indy7_gazebo indy_gazebo.launch.py

  # 실험 환경 world + Indy7
  # pick_place에서는 실물 선반 높이에 맞춰 로봇 spawn z가 자동으로 올라간다.
  ros2 launch indy7_gazebo indy_gazebo.launch.py gazebo_env:=pick_place

  # MoveIt까지 같이 쓰는 경우는 indy7_moveit 쪽 통합 launch 사용
  ros2 launch indy7_moveit indy_moveit_gazebo.launch.py gazebo_env:=pick_place
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    description_package = FindPackageShare("indy7_description")
    gazebo_package = FindPackageShare("indy7_gazebo")

    # Initialize Arguments
    name = LaunchConfiguration("name")
    indy_type = LaunchConfiguration("indy_type")
    indy_eye = LaunchConfiguration("indy_eye")
    prefix = LaunchConfiguration("prefix")
    launch_rviz = LaunchConfiguration("launch_rviz")
    gazebo_env = LaunchConfiguration("gazebo_env")
    robot_spawn_z = LaunchConfiguration("robot_spawn_z")

    # 6축 arm controller yaml 선택
    initial_joint_controllers = PathJoinSubstitution(
        [gazebo_package, "controller", "indy7_controllers.yaml"]
    )


    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([description_package, "urdf", "indy7_with_gripper.urdf.xacro"]),
            " ",
            "name:=",
            name,
            " ",
            "indy_type:=",
            indy_type,
            " ",
            "indy_eye:=",
            indy_eye,
            " ",
            "prefix:=",
            prefix,
            " ",
            "sim_gazebo:=true",
            " ",
            "simulation_controllers:=",
            initial_joint_controllers,
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    rviz_config_file = PathJoinSubstitution(
        [description_package, "rviz_config", "indy.rviz"]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            robot_description,
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    joint_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "-c",
            "/controller_manager",
        ],
        output="screen",
    )

    # 추가: gripper controller spawner
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "-c",
            "/controller_manager"
        ],
        output="screen",
    )

    # Gazebo 실행
    gazebo_launch_arguments = {}
    gazebo_env_name = gazebo_env.perform(context).strip()
    if gazebo_env_name not in ("", "default", "none"):
        if gazebo_env_name.endswith(".world"):
            gazebo_env_name = gazebo_env_name[:-6]
        gazebo_launch_arguments["world"] = PathJoinSubstitution(
            [gazebo_package, "worlds", f"{gazebo_env_name}.world"]
        )

    robot_spawn_z_value = robot_spawn_z.perform(context).strip()
    if robot_spawn_z_value in ("", "auto"):
        robot_spawn_z_value = "0.634" if gazebo_env_name == "pick_place" else "0.0"

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
        launch_arguments=gazebo_launch_arguments.items(),
    )

    # Gazebo에 로봇 spawn
    gazebo_spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_indy",
        arguments=[
            "-entity",
            "indy",
            "-topic",
            "robot_description",
            "-z",
            robot_spawn_z_value,
        ],
        output="screen",
    )

    rviz_node = Node(
        condition=IfCondition(launch_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # spawn 끝난 뒤 joint_state_broadcaster 실행
    delay_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo_spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    # joint_state_broadcaster 실행 뒤 arm controller 실행
    delay_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_controller_spawner],
        )
    )

    # arm controller 실행 뒤 gripper controller 실행
    delay_gripper_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    # arm controller 실행 뒤 rviz 실행
    delay_rviz2_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_controller_spawner,
            on_exit=[rviz_node],
        )
    )

    nodes_to_start = [
        gazebo,
        gazebo_spawn_robot,
        robot_state_publisher_node,

        delay_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner,
        delay_gripper_controller_spawner,
        delay_rviz2_spawner,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "name",
            default_value="indy",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "indy_type",
            default_value="indy7",
            description="Type of Indy robot.",
            choices=[
                "indy7",
                "indy7_v2",
                "indy7_v3",
                "indy12",
                "indy12_v2",
                "indyrp2",
                "indyrp2_v2",
                "icon7l",
                "icon3",
                "nuri3s",
                "nuri4s",
                "nuri7c",
                "nuri20c",
                "opti5",
            ],
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
            "prefix",
            default_value="",
            description=(
                "Prefix of the joint names, useful for multi-robot setup. "
                "If changed, joint names in controller configuration must also be updated."
            ),
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz?",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "gazebo_env",
            default_value="",
            description=(
                "Gazebo environment name. Empty/default uses the normal "
                "Gazebo world; pick_place loads worlds/pick_place.world."
            ),
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_spawn_z",
            default_value="auto",
            description=(
                "Robot spawn height in Gazebo. 'auto' uses 0.634 for "
                "pick_place and 0.0 for the normal world."
            ),
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
