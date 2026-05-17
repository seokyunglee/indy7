"""
Indy7 Pick & Place Task Node
============================
YAML/JSON에 저장된 목표 좌표를 읽어 8단계 pick-and-place 사이클을 실행한다.

Pick 위치는 YAML의 고정 좌표를 사용하고, place/pass_place 위치는 YAML 또는
JSON에서 읽는다. pick_above, lift, place_above 같은 중간 pose는 기준 좌표의
z값만 올려 계산한다.

실행 방법:
  # MoveIt(move_group), controller, gripper node가 먼저 실행되어 있어야 한다.
  ros2 launch indy7_task task_only.launch.py

  # JSON pass_place를 사용하려면:
  ros2 launch indy7_task task_only.launch.py use_pass_place:=true
"""

import time
from copy import deepcopy
from threading import Thread

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from indy7_task.pose_loader import PoseLoader
from indy7_task.gripper_client import GripperClient
from indy7_task.moveit_client import Indy7MoveItClient


def as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class Indy7TaskNode(Node):
    """Indy7 task-level pick-and-place 실행 노드."""

    def __init__(self):
        super().__init__("indy7_task_node")

        # ------------------------------------------------------
        #  Package config paths and launch parameters
        # ------------------------------------------------------
        package_share = get_package_share_directory("indy7_task")
        default_task_poses = (
            f"{package_share}/config/task_poses.yaml"
        )
        default_pass_place_goal = (
            f"{package_share}/config/pass_place_goal.json"
        )

        self.declare_parameter("task_poses_path", default_task_poses)
        self.declare_parameter("pass_place_goal_path", default_pass_place_goal)
        self.declare_parameter("use_pass_place", False)
        self.declare_parameter("auto_start", True)
        self.declare_parameter("gripper_open_service", "/gripper/open")
        self.declare_parameter("gripper_close_service", "/gripper/close")
        self.declare_parameter("gripper_state_service", "/gripper/state")

        self.task_poses_path = self.get_parameter("task_poses_path").value
        self.pass_place_goal_path = self.get_parameter(
            "pass_place_goal_path"
        ).value
        self.use_pass_place = as_bool(
            self.get_parameter("use_pass_place").value
        )
        self.auto_start = as_bool(self.get_parameter("auto_start").value)

        if not self.task_poses_path:
            raise ValueError("Parameter 'task_poses_path' is empty")

        # ------------------------------------------------------
        #  Helpers: pose loading, gripper services, MoveIt client
        # ------------------------------------------------------
        self.pose_loader = PoseLoader(
            node=self,
            yaml_path=self.task_poses_path,
            json_path=self.pass_place_goal_path,
        )

        self.gripper = GripperClient(
            self,
            open_service=self.get_parameter("gripper_open_service").value,
            close_service=self.get_parameter("gripper_close_service").value,
            state_service=self.get_parameter(
                "gripper_state_service"
            ).value,
        )
        self.moveit = Indy7MoveItClient(self)

        self.step_wait_sec = float(
            self.pose_loader.get_task_param("step_wait_sec", 0.5)
        )

        self.get_logger().info("Indy7 task node initialized")

    # ----------------------------------------------------------
    #  Step timing / motion wrappers
    # ----------------------------------------------------------
    def wait_step(self, label: str):
        """각 단계 사이에 고정 대기 시간을 둔다."""
        self.get_logger().info(f"[STEP] {label}")
        time.sleep(self.step_wait_sec)

    def move_to_pose(self, pose_name, label=None):
        """YAML에 정의된 이름 pose로 seeded IK 기반 이동."""
        label = label or pose_name
        pose = self.pose_loader.get_pose(pose_name)
        if not self.moveit.go_smooth(pose, label):
            raise RuntimeError(f"Failed to move to {label}")

    def move_to_pose_stamped(self, pose, label):
        """이미 계산된 PoseStamped 목표로 seeded IK 기반 이동."""
        if not self.moveit.go_smooth(pose, label):
            raise RuntimeError(f"Failed to move to {label}")

    # ----------------------------------------------------------
    #  Derived pick/place poses
    # ----------------------------------------------------------
    def pose_with_z(self, source_pose, z_value):
        """source_pose의 x/y/orientation을 유지하고 z만 교체한다."""
        pose = deepcopy(source_pose)
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.z = float(z_value)
        return pose

    def pose_z_or_default(self, pose_name, fallback):
        """기존 pose가 있으면 그 z값을 중간 pose 기본값으로 사용한다."""
        try:
            return self.pose_loader.get_pose(pose_name).pose.position.z
        except KeyError:
            return fallback

    def get_place_target_pose(self):
        """use_pass_place에 따라 YAML place 또는 JSON pass_place를 선택한다."""
        if self.use_pass_place:
            return self.pose_loader.get_pass_place_pose(), "pass_place"
        return self.pose_loader.get_pose("place"), "place"

    # ----------------------------------------------------------
    #  Main 8-step pick-and-place sequence
    # ----------------------------------------------------------
    def run_pick_and_place(self):
        self.get_logger().info("=== Pick and Place Task Start ===")

        if not self.gripper.wait_for_servers(timeout_sec=3.0):
            raise RuntimeError("Gripper services are not fully available")
        if not self.moveit.wait_for_servers(timeout_sec=30.0):
            raise RuntimeError("MoveIt action/service servers are unavailable")
        if not self.moveit.wait_for_joint_state(timeout_sec=10.0):
            raise RuntimeError("joint_states are unavailable")

        pick_pose = self.pose_loader.get_pose("pick")
        place_pose, place_label = self.get_place_target_pose()

        pick_above_z = float(
            self.pose_loader.get_task_param(
                "pick_above_z",
                self.pose_z_or_default("pre_pick", 0.30),
            )
        )
        lift_z = float(
            self.pose_loader.get_task_param(
                "lift_z",
                self.pose_z_or_default("lift", 0.35),
            )
        )
        place_above_z = float(
            self.pose_loader.get_task_param(
                "place_above_z",
                self.pose_z_or_default("pre_place", 0.35),
            )
        )

        pick_above_pose = self.pose_with_z(pick_pose, pick_above_z)
        lift_pose = self.pose_with_z(pick_pose, lift_z)
        place_above_pose = self.pose_with_z(place_pose, place_above_z)

        # 1단계: ready 자세로 이동하고 그리퍼를 연다.
        self.wait_step("ready + gripper open")
        self.move_to_pose("ready")
        if not self.gripper.open():
            raise RuntimeError("Failed to open gripper")

        # 2단계: pick 상공으로 접근한 뒤 실제 pick 위치로 하강한다.
        self.wait_step("approach pick")
        self.move_to_pose_stamped(pick_above_pose, "pick_above")
        self.move_to_pose_stamped(pick_pose, "pick")

        # 3단계: 그리퍼를 닫아 물체를 잡는다.
        self.wait_step("gripper close")
        if not self.gripper.close():
            raise RuntimeError("Failed to close gripper")

        # 4단계: pick 위치의 x/y를 유지한 채 z만 올려 들어올린다.
        self.wait_step("lift")
        self.move_to_pose_stamped(lift_pose, "lift")

        # 5단계: place/pass_place 위치의 상공으로 이동한다.
        self.wait_step(f"move above {place_label}")
        self.move_to_pose_stamped(place_above_pose, f"{place_label}_above")

        # 6단계: place/pass_place 위치로 하강한다.
        self.wait_step(f"descend to {place_label}")
        self.move_to_pose_stamped(place_pose, place_label)

        # 7단계: 그리퍼를 열어 물체를 놓는다.
        self.wait_step("gripper open release")
        if not self.gripper.open():
            raise RuntimeError("Failed to open gripper for release")

        # 8단계: 상공으로 후퇴한 뒤 ready 자세로 복귀한다.
        self.wait_step("retreat and return ready")
        self.move_to_pose_stamped(place_above_pose, f"retreat/{place_label}")
        self.move_to_pose("ready")

        self.get_logger().info("=== Pick and Place Task Done ===")


def main(args=None):
    """ROS entry point."""
    rclpy.init(args=args)

    node = Indy7TaskNode()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        time.sleep(2.0)

        if node.auto_start:
            node.run_pick_and_place()
        else:
            node.get_logger().info(
                "auto_start is false. Node is ready, but task will not start."
            )

    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"Task failed: {e}")
    finally:
        node.get_logger().info("Shutting down indy7_task_node")
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        executor_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
