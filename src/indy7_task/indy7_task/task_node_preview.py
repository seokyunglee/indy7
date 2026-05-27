"""
보완 필요함
Preview-first Indy7 task node.

기존 task_node.py/task_node_servo.py는 건드리지 않고, 실물 확인용으로
각 segment를 plan-only로 만든 뒤 RViz Marker로 TCP 경로를 보여주고
사용자 확인 후 실행한다.
"""

import copy
import termios
import time
import tty
import yaml
from threading import Thread

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from indy7_task.gripper_client import GripperClient
from indy7_task.moveit_preview_client import PreviewMoveItClient
from indy7_task.planning_scene_client import Indy7PlanningSceneClient
from indy7_task.pose_loader import PoseLoader
from indy7_task.task_node import as_bool, as_float_tuple


class Indy7TaskPreviewNode(Node):
    """Plan preview, keyboard confirmation, and cautious execution node."""

    def __init__(self):
        super().__init__("indy7_task_preview_node")

        package_share = get_package_share_directory("indy7_task")
        self.declare_parameter(
            "task_poses_path",
            f"{package_share}/config/task_poses.yaml",
        )
        self.declare_parameter(
            "pass_place_goal_path",
            f"{package_share}/config/pass_place_goal.json",
        )
        self.declare_parameter(
            "task_segments_path",
            f"{package_share}/config/task_segments.yaml",
        )
        self.declare_parameter("use_pass_place", False)
        self.declare_parameter("auto_start", True)
        self.declare_parameter("confirm_before_execute", True)
        self.declare_parameter("allow_pose_fallback", False)
        self.declare_parameter("use_planning_scene", True)
        self.declare_parameter("clear_scene_on_start", True)
        self.declare_parameter("pick_shelf_collision_id", "pick_shelf")
        self.declare_parameter("pick_shelf_top_center", [0.65, 0.0, 0.16])
        self.declare_parameter("pick_shelf_dimensions", [0.32, 0.30, 0.05])
        self.declare_parameter("gripper_open_service", "/gripper/open")
        self.declare_parameter("gripper_close_service", "/gripper/close")
        self.declare_parameter("gripper_state_service", "/gripper/state")

        self.task_poses_path = self.get_parameter("task_poses_path").value
        self.pass_place_goal_path = self.get_parameter(
            "pass_place_goal_path"
        ).value
        self.task_segments_path = self.get_parameter("task_segments_path").value
        self.use_pass_place = as_bool(
            self.get_parameter("use_pass_place").value
        )
        self.auto_start = as_bool(self.get_parameter("auto_start").value)
        self.confirm_before_execute = as_bool(
            self.get_parameter("confirm_before_execute").value
        )
        self.allow_pose_fallback = as_bool(
            self.get_parameter("allow_pose_fallback").value
        )
        self.use_planning_scene = as_bool(
            self.get_parameter("use_planning_scene").value
        )
        self.clear_scene_on_start = as_bool(
            self.get_parameter("clear_scene_on_start").value
        )
        self.pick_shelf_collision_id = self.get_parameter(
            "pick_shelf_collision_id"
        ).value
        self.pick_shelf_top_center = as_float_tuple(
            self.get_parameter("pick_shelf_top_center").value,
            3,
            "pick_shelf_top_center",
        )
        self.pick_shelf_dimensions = as_float_tuple(
            self.get_parameter("pick_shelf_dimensions").value,
            3,
            "pick_shelf_dimensions",
        )

        self.pose_loader = PoseLoader(
            node=self,
            yaml_path=self.task_poses_path,
            json_path=self.pass_place_goal_path,
        )
        self.gripper = GripperClient(
            self,moveit
            open_service=self.get_parameter("gripper_open_service").value,
            close_service=self.get_parameter("gripper_close_service").value,
            state_service=self.get_parameter(
                "gripper_state_service"
            ).value,
        )
        self.moveit = PreviewMoveItClient(self)
        self.scene = Indy7PlanningSceneClient(self)

        self.segment_config = self._load_segment_config()
        self.joint_poses = self.segment_config.get("joint_poses", {}) or {}
        self.sequence = self.segment_config.get("sequence", []) or []
        if not self.sequence:
            raise ValueError("task_segments.yaml 안에 sequence가 없습니다")

        self.get_logger().info(
            "Indy7 preview task node 초기화 완료: "
            f"{len(self.sequence)} segments"
        )

    # ----------------------------------------------------------
    #  Config and setup
    # ----------------------------------------------------------
    def _load_segment_config(self):
        if not self.task_segments_path:
            raise ValueError("Parameter 'task_segments_path' is empty")
        with open(self.task_segments_path, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
        if not isinstance(data, dict):
            raise ValueError("task_segments.yaml root must be a mapping")
        return data

    def setup_planning_scene(self):
        if not self.use_planning_scene:
            self.get_logger().info("Planning scene 구성이 비활성화되어 있습니다")
            return

        if not self.scene.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("Planning scene 서비스를 사용할 수 없습니다")

        if self.clear_scene_on_start:
            if not self.scene.clear_scene(timeout_sec=5.0):
                raise RuntimeError("Planning scene 초기화 실패")

        ok = self.scene.add_box_by_top_center(
            self.pick_shelf_collision_id,
            top_center=self.pick_shelf_top_center,
            dimensions=self.pick_shelf_dimensions,
        )
        if not ok:
            raise RuntimeError("Planning scene pick shelf 구성 실패")

        self.get_logger().info(
            f"Planning scene 준비 완료: {self.pick_shelf_collision_id}"
        )

    # ----------------------------------------------------------
    #  Keyboard confirmation
    # ----------------------------------------------------------
    def _open_keyboard_input(self):
        try:
            return open("/dev/tty", "r", encoding="utf-8")
        except OSError:
            return None

    def _prompt(self, label, plan_ok=True):
        if not self.confirm_before_execute and plan_ok:
            return "execute"

        if plan_ok:
            self.get_logger().info(
                f"{label}: SPACE=execute, r=replan, s=skip, q=stop"
            )
            allowed = {
                " ": "execute",
                "r": "replan",
                "s": "skip",
                "q": "stop",
            }
        else:
            self.get_logger().info(f"{label}: r=replan, s=skip, q=stop")
            allowed = {
                "r": "replan",
                "s": "skip",
                "q": "stop",
            }

        input_file = self._open_keyboard_input()
        if input_file is None:
            self.get_logger().warn(
                "No interactive terminal detected. Preview task will skip "
                "execution unless confirm_before_execute is false."
            )
            return "skip"

        old_settings = termios.tcgetattr(input_file)
        try:
            tty.setcbreak(input_file.fileno())
            while rclpy.ok():
                key = input_file.read(1)
                key = key.lower()
                if key in allowed:
                    return allowed[key]
        finally:
            termios.tcsetattr(input_file, termios.TCSADRAIN, old_settings)
            input_file.close()

        return "stop"

    # ----------------------------------------------------------
    #  Pose and joint target helpers
    # ----------------------------------------------------------
    def _get_pose(self, name, orientation_from=None):
        if self.use_pass_place and name in ("pass", "pass_place"):
            pose = self.pose_loader.get_pass_place_pose()
        else:
            pose = self.pose_loader.get_pose(name)

        pose = copy.deepcopy(pose)
        if orientation_from:
            reference = self.pose_loader.get_pose(orientation_from)
            pose.pose.orientation = copy.deepcopy(reference.pose.orientation)
        return pose

    def _segment_pose(self, segment):
        target = segment.get("target")
        if not target:
            raise ValueError(f"{segment.get('name', 'segment')}: target이 없습니다")
        return self._get_pose(target, orientation_from=segment.get("orientation_from"))

    def _segment_waypoints(self, segment):
        names = segment.get("waypoints", [])
        if not names:
            raise ValueError(f"{segment.get('name', 'segment')}: waypoints가 없습니다")

        poses = []
        for item in names:
            if isinstance(item, str):
                pose_name = item
                orientation_from = segment.get("orientation_from")
            else:
                pose_name = item.get("pose")
                orientation_from = item.get(
                    "orientation_from",
                    segment.get("orientation_from"),
                )
            poses.append(
                self._get_pose(
                    pose_name,
                    orientation_from=orientation_from,
                ).pose
            )
        return poses

    def _joint_values_from_target(self, target):
        if target not in self.joint_poses:
            raise ValueError(f"joint pose가 없습니다: {target}")

        values = self.joint_poses[target]
        if isinstance(values, dict):
            missing = [
                name for name in self.moveit.joint_names
                if name not in values
            ]
            if missing:
                raise ValueError(
                    f"{target}: joint pose에 빠진 joint가 있습니다: "
                    f"{', '.join(missing)}"
                )
            return {
                name: float(values[name])
                for name in self.moveit.joint_names
            }

        if not isinstance(values, (list, tuple)):
            raise ValueError(f"{target}: joint pose는 list 또는 dict여야 합니다")
        if len(values) != len(self.moveit.joint_names):
            raise ValueError(
                f"{target}: joint pose 길이가 {len(self.moveit.joint_names)}"
                f"이어야 합니다"
            )

        return {
            name: float(values[index])
            for index, name in enumerate(self.moveit.joint_names)
        }

    # ----------------------------------------------------------
    #  Segment planning and execution
    # ----------------------------------------------------------
    def _plan_segment(self, segment):
        label = segment.get("name", "segment")
        segment_type = segment.get("type", "pose")

        if segment_type == "pose":
            pose = self._segment_pose(segment)
            return self.moveit.plan_pose_for_preview(
                pose,
                label=label,
                allow_pose_fallback=self.allow_pose_fallback,
            )

        if segment_type == "joint":
            target = segment.get("target")
            joint_values = self._joint_values_from_target(target)
            return self.moveit.plan_joint_values_for_preview(
                joint_values,
                label=label,
            )

        if segment_type == "cartesian":
            poses = self._segment_waypoints(segment)
            return self.moveit.plan_cartesian_poses_for_preview(
                poses,
                label=label,
            )

        raise ValueError(f"{label}: 알 수 없는 segment type={segment_type}")

    def _run_motion_segment(self, segment, index, total):
        label = segment.get("name", f"segment_{index}")
        self.get_logger().info(f"[SEGMENT {index}/{total}] {label}")

        while rclpy.ok():
            success, trajectory = self._plan_segment(segment)
            if not success or trajectory is None:
                action = self._prompt(label, plan_ok=False)
                if action == "replan":
                    continue
                if action == "skip":
                    self.get_logger().warn(f"{label}: skipped")
                    return
                raise KeyboardInterrupt

            self.moveit.preview_trajectory(trajectory, label)
            action = self._prompt(label, plan_ok=True)
            if action == "replan":
                continue
            if action == "skip":
                self.get_logger().warn(f"{label}: skipped")
                return
            if action == "stop":
                raise KeyboardInterrupt

            if not self.moveit.execute_trajectory(trajectory):
                raise RuntimeError(f"{label}: trajectory 실행 실패")
            self.get_logger().info(f"{label}: 실행 완료")
            time.sleep(0.2)
            return

    def _run_gripper_segment(self, segment, index, total):
        command = segment.get("command")
        label = segment.get("name", f"gripper_{command}")
        self.get_logger().info(f"[SEGMENT {index}/{total}] {label}")

        action = self._prompt(label, plan_ok=True)
        if action == "skip":
            self.get_logger().warn(f"{label}: skipped")
            return
        if action in ("stop", "replan"):
            raise KeyboardInterrupt

        if command == "open":
            ok = self.gripper.open()
        elif command == "close":
            ok = self.gripper.close()
        elif command == "state":
            ok = self.gripper.state()
        else:
            raise ValueError(f"{label}: 알 수 없는 gripper command={command}")

        if not ok:
            raise RuntimeError(f"{label}: gripper command 실패")

    def run(self):
        self.get_logger().info("=== Preview Pick and Pass 작업 시작 ===")

        if not self.gripper.wait_for_servers(timeout_sec=3.0):
            raise RuntimeError("Gripper 서비스가 모두 준비되지 않았습니다")
        if not self.moveit.wait_for_servers(timeout_sec=30.0):
            raise RuntimeError("MoveIt 액션/서비스 서버를 사용할 수 없습니다")
        if not self.moveit.wait_for_joint_state(timeout_sec=10.0):
            raise RuntimeError("joint_states를 사용할 수 없습니다")
        self.setup_planning_scene()

        total = len(self.sequence)
        for index, segment in enumerate(self.sequence, start=1):
            segment_type = segment.get("type", "pose")
            if segment_type == "gripper":
                self._run_gripper_segment(segment, index, total)
            else:
                self._run_motion_segment(segment, index, total)

        self.get_logger().info("=== Preview Pick and Pass 작업 완료 ===")


def main(args=None):
    rclpy.init(args=args)
    node = Indy7TaskPreviewNode()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        if node.auto_start:
            node.run()
        else:
            node.get_logger().info(
                "auto_start is false. Node is ready, but task will not start."
            )
    except KeyboardInterrupt:
        node.get_logger().info("Preview task interrupted")
    except Exception as exc:
        node.get_logger().error(f"Preview task failed: {exc}")
    finally:
        node.get_logger().info("Shutting down indy7_task_preview_node")
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        executor_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
