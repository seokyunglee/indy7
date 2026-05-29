"""
Indy7 Pick & Place Task Node
============================
YAML/JSON에 저장된 목표 좌표를 읽어 8단계 pick-and-pass 사이클을 실행한다.

Pick/pass 위치는 YAML의 고정 좌표를 사용하고, use_pass_place가 true이면
pass 목표만 JSON에서 읽는다. ready_pick/ready_pass는 joint target으로
보내고, pre_pick과 pick/pass 목표는 YAML/JSON의 pose를 그대로 사용한다.

실행 방법:
  # MoveIt(move_group), controller, gripper node가 먼저 실행되어 있어야 한다.
  ros2 launch indy7_task task_only.launch.py

  # JSON pass_place를 사용하려면:
  ros2 launch indy7_task task_only.launch.py use_pass_place:=true
"""

import time
import yaml
from threading import Thread

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from indy7_task.pose_loader import PoseLoader
from indy7_task.gripper_client import GripperClient
from indy7_task.moveit_client import Indy7MoveItClient
from indy7_task.planning_scene_client import Indy7PlanningSceneClient


def as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def as_float_tuple(value, expected_len, name):
    """ROS parameter/list/string 값을 float tuple로 정규화한다."""
    if isinstance(value, str):
        value = yaml.safe_load(value)

    if not isinstance(value, (list, tuple)):
        raise ValueError(f"Parameter '{name}' must be a list/tuple")
    if len(value) != expected_len:
        raise ValueError(
            f"Parameter '{name}' must have {expected_len} values"
        )

    return tuple(float(item) for item in value)


class Indy7TaskNode(Node):
    """Indy7 작업 단위 pick-and-pass 실행 노드."""

    def __init__(self):
        super().__init__("indy7_task_node")

        # ------------------------------------------------------
        #  패키지 설정 경로와 launch 파라미터
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
        self.declare_parameter("use_planning_scene", True)
        self.declare_parameter("clear_scene_on_start", True)
        self.declare_parameter("pick_shelf_collision_id", "pick_shelf")
        self.declare_parameter("pick_shelf_top_center", [0.0, -0.72, 0.081])
        self.declare_parameter("pick_shelf_dimensions", [0.32, 0.30, 0.05])
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

        if not self.task_poses_path:
            raise ValueError("Parameter 'task_poses_path' is empty")

        # ------------------------------------------------------
        #  헬퍼: pose 로딩, 그리퍼 서비스, MoveIt/scene 클라이언트
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
        self.scene = Indy7PlanningSceneClient(self)

        self.step_wait_sec = float(
            self.pose_loader.get_task_param("step_wait_sec", 0.5)
        )

        self.get_logger().info("Indy7 task node 초기화 완료")

    # ----------------------------------------------------------
    #  단계 대기 / 이동 래퍼
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

    def move_to_joint_target(self, target_name, label=None):
        """YAML에 정의된 joint target으로 이동한다."""
        label = label or target_name
        target = self.pose_loader.get_joint_target(target_name)
        target_by_name = dict(zip(target["joint_names"], target["positions"]))

        missing = [
            joint_name
            for joint_name in self.moveit.joint_names
            if joint_name not in target_by_name
        ]
        if missing:
            raise RuntimeError(
                f"Joint target '{target_name}' is missing joints: "
                f"{', '.join(missing)}"
            )

        joint_values = {
            joint_name: target_by_name[joint_name]
            for joint_name in self.moveit.joint_names
        }
        plan_success, trajectory = self.moveit.plan_to_joint_goal(joint_values)
        if not plan_success or trajectory is None:
            raise RuntimeError(f"Failed to plan to {label}")
        if not self.moveit.check_trajectory_safety(trajectory, label=label):
            raise RuntimeError(f"Unsafe trajectory to {label}")
        if not self.moveit.execute_trajectory(trajectory):
            raise RuntimeError(f"Failed to execute {label}")

        self.get_logger().info(f"{label} 이동 완료")

    # ----------------------------------------------------------
    #  Planning scene 구성
    # ----------------------------------------------------------
    def setup_planning_scene(self):
        """MoveIt planning scene에는 pick 선반만 정적 장애물로 등록한다.
        """
        if not self.use_planning_scene:
            self.get_logger().info("Planning scene 구성이 비활성화되어 있습니다")
            return

        if not self.scene.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("Planning scene 서비스를 사용할 수 없습니다")

        if self.clear_scene_on_start:
            if not self.scene.clear_scene(timeout_sec=5.0):
                raise RuntimeError("Planning scene 초기화 실패")

        shelf_ok = self.scene.add_box_by_top_center(
            self.pick_shelf_collision_id,
            top_center=self.pick_shelf_top_center,
            dimensions=self.pick_shelf_dimensions,
        )
        if not shelf_ok:
            raise RuntimeError("Planning scene pick shelf 구성 실패")

        self.get_logger().info(
            f"Planning scene 준비 완료: {self.pick_shelf_collision_id}"
        )

    def get_pass_target_pose(self):
        """use_pass_place에 따라 YAML pass 또는 JSON pass_place를 선택한다."""
        if self.use_pass_place:
            return self.pose_loader.get_pass_place_pose(), "pass_place"
        return self.pose_loader.get_pose("pass"), "pass"

    # ----------------------------------------------------------
    #  8단계 pick-and-pass 시퀀스
    # ----------------------------------------------------------
    def run_pick_and_place(self):
        self.get_logger().info("=== Pick and Pass 작업 시작 ===")

        if not self.gripper.wait_for_servers(timeout_sec=3.0):
            raise RuntimeError("Gripper 서비스가 모두 준비되지 않았습니다")
        if not self.moveit.wait_for_servers(timeout_sec=30.0):
            raise RuntimeError("MoveIt 액션/서비스 서버를 사용할 수 없습니다")
        if not self.moveit.wait_for_joint_state(timeout_sec=10.0):
            raise RuntimeError("joint_states를 사용할 수 없습니다")
        self.setup_planning_scene()

        pass_pose, pass_label = self.get_pass_target_pose()

        # 1단계: pick 기준 ready 자세로 이동하고 그리퍼를 연다.
        self.wait_step("ready_pick + gripper open")
        self.move_to_joint_target("ready_pick")
        if not self.gripper.open():
            raise RuntimeError("그리퍼 열기 실패")

        # 2단계: pick 상공으로 접근한 뒤 실제 pick 위치로 간다.
        self.wait_step("pre_pick -> pick")
        self.move_to_pose("pre_pick")
        self.move_to_pose("pick")

        # 3단계: 그리퍼를 닫아 물체를 잡는다.
        self.wait_step("gripper close")
        if not self.gripper.close():
            raise RuntimeError("그리퍼 닫기 실패")

        # 4단계: pre_pick을 거쳐 pick 기준 ready 자세로 돌아온다.
        self.wait_step("pre_pick -> ready_pick")
        self.move_to_pose("pre_pick")
        self.move_to_joint_target("ready_pick")

        # 5단계: pass 기준 ready joint 자세로 이동한다.
        self.wait_step("ready_pass")
        self.move_to_joint_target("ready_pass")

        # 6단계: ready_pass joint 자세에서 pass 목표 pose로 바로 간다.
        self.wait_step(f"ready_pass -> {pass_label}")
        self.move_to_pose_stamped(pass_pose, pass_label)

        # 7단계: 그리퍼를 열어 물체를 놓는다.
        self.wait_step("gripper open release")
        if not self.gripper.open():
            raise RuntimeError("물체 release를 위한 그리퍼 열기 실패")

        # 8단계: pass ready joint 자세를 거쳐 pick ready로 돌아온다.
        self.wait_step("ready_pass -> ready_pick")
        self.move_to_joint_target("ready_pass")
        self.move_to_joint_target("ready_pick")

        self.get_logger().info("=== Pick and Pass 작업 완료 ===")


def main(args=None):
    """ROS 실행 진입점."""
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
                "auto_start가 false입니다. 노드는 준비됐지만 작업은 시작하지 않습니다."
            )

    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f"작업 실패: {e}")
    finally:
        node.get_logger().info("indy7_task_node 종료 중")
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        executor_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
