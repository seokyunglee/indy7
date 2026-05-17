from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    task_executable = LaunchConfiguration("task_executable")
    task_poses_path = LaunchConfiguration("task_poses_path")
    pass_place_goal_path = LaunchConfiguration("pass_place_goal_path")
    use_pass_place = LaunchConfiguration("use_pass_place")
    auto_start = LaunchConfiguration("auto_start")
    group_name = LaunchConfiguration("group_name")
    base_link_name = LaunchConfiguration("base_link_name")
    end_effector_name = LaunchConfiguration("end_effector_name")
    max_velocity = LaunchConfiguration("max_velocity")
    max_acceleration = LaunchConfiguration("max_acceleration")
    planning_time = LaunchConfiguration("planning_time")
    num_planning_attempts = LaunchConfiguration("num_planning_attempts")
    position_tolerance = LaunchConfiguration("position_tolerance")
    orientation_tolerance = LaunchConfiguration("orientation_tolerance")
    gripper_open_service = LaunchConfiguration("gripper_open_service")
    gripper_close_service = LaunchConfiguration("gripper_close_service")
    gripper_state_service = LaunchConfiguration("gripper_state_service")

    return LaunchDescription([
        DeclareLaunchArgument(
            "task_executable",
            default_value="task_node",
            description="Use task_node or task_node_servo.",
        ),
        DeclareLaunchArgument(
            "task_poses_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("indy7_task"),
                "config",
                "task_poses.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "pass_place_goal_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("indy7_task"),
                "config",
                "pass_place_goal.json",
            ]),
        ),
        DeclareLaunchArgument("use_pass_place", default_value="false"),
        DeclareLaunchArgument("auto_start", default_value="true"),
        DeclareLaunchArgument("group_name", default_value="indy_manipulator"),
        DeclareLaunchArgument("base_link_name", default_value="link0"),
        DeclareLaunchArgument("end_effector_name", default_value="tcp"),
        DeclareLaunchArgument("max_velocity", default_value="0.2"),
        DeclareLaunchArgument("max_acceleration", default_value="0.2"),
        DeclareLaunchArgument("planning_time", default_value="5.0"),
        DeclareLaunchArgument("num_planning_attempts", default_value="5"),
        DeclareLaunchArgument("position_tolerance", default_value="0.01"),
        DeclareLaunchArgument("orientation_tolerance", default_value="0.01"),
        DeclareLaunchArgument(
            "gripper_open_service",
            default_value="/gripper/open",
        ),
        DeclareLaunchArgument(
            "gripper_close_service",
            default_value="/gripper/close",
        ),
        DeclareLaunchArgument(
            "gripper_state_service",
            default_value="/gripper/state",
        ),
        Node(
            package="indy7_task",
            executable=task_executable,
            name="indy7_task_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "task_poses_path": task_poses_path,
                    "pass_place_goal_path": pass_place_goal_path,
                    "use_pass_place": ParameterValue(
                        use_pass_place,
                        value_type=bool,
                    ),
                    "auto_start": ParameterValue(auto_start, value_type=bool),
                    "group_name": group_name,
                    "base_link_name": base_link_name,
                    "end_effector_name": end_effector_name,
                    "max_velocity": ParameterValue(
                        max_velocity,
                        value_type=float,
                    ),
                    "max_acceleration": ParameterValue(
                        max_acceleration,
                        value_type=float,
                    ),
                    "planning_time": ParameterValue(
                        planning_time,
                        value_type=float,
                    ),
                    "num_planning_attempts": ParameterValue(
                        num_planning_attempts,
                        value_type=int,
                    ),
                    "position_tolerance": ParameterValue(
                        position_tolerance,
                        value_type=float,
                    ),
                    "orientation_tolerance": ParameterValue(
                        orientation_tolerance,
                        value_type=float,
                    ),
                    "gripper_open_service": gripper_open_service,
                    "gripper_close_service": gripper_close_service,
                    "gripper_state_service": gripper_state_service,
                }
            ],
        ),
    ])
