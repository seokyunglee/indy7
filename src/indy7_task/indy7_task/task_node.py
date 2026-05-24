"""
Indy7 Pick & Place Task Node
============================
YAML/JSON에 저장된 목표 좌표를 읽어 8단계 pick-and-place 사이클을 실행한다.

Pick 위치는 YAML의 고정 좌표를 사용하고, place/pass_place 위치는 YAML 또는
JSON에서 읽는다. pre_pick, lift, pre_place 같은 중간 pose도 YAML에 정의된
position/orientation을 그대로 사용한다.

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
    """Indy7 작업 단위 pick-and-place 실행 노드."""

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
        self.declare_parameter("pick_shelf_top_center", [0.65, 0.0, 0.16])
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

    # ----------------------------------------------------------
    #  Planning scene 구성
    # ----------------------------------------------------------
    def setup_planning_scene(self):
        """MoveIt planning scene에는 pick 선반만 정적 장애물로 등록한다.

        Gazebo/실물 환경의 상세 물체와 grasp 대상은 MoveIt에 중복
        등록하지 않는다. MoveIt은 pick 선반을 피하는 궤적 생성만 맡고,
        물체 상호작용은 Gazebo/실물 쪽이 담당한다.
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

    def get_place_target_pose(self):
        """use_pass_place에 따라 YAML place 또는 JSON pass_place를 선택한다."""
        if self.use_pass_place:
            return self.pose_loader.get_pass_place_pose(), "pass_place"
        return self.pose_loader.get_pose("place"), "place"

    # ----------------------------------------------------------
    #  8단계 pick-and-place 시퀀스
    # ----------------------------------------------------------
    def run_pick_and_place(self):
        self.get_logger().info("=== Pick and Place 작업 시작 ===")

        if not self.gripper.wait_for_servers(timeout_sec=3.0):
            raise RuntimeError("Gripper 서비스가 모두 준비되지 않았습니다")
        if not self.moveit.wait_for_servers(timeout_sec=30.0):
            raise RuntimeError("MoveIt 액션/서비스 서버를 사용할 수 없습니다")
        if not self.moveit.wait_for_joint_state(timeout_sec=10.0):
            raise RuntimeError("joint_states를 사용할 수 없습니다")
        self.setup_planning_scene()

        pick_pose = self.pose_loader.get_pose("pick")
        pick_above_pose = self.pose_loader.get_pose("pre_pick")
        lift_pose = self.pose_loader.get_pose("lift")
        place_above_pose = self.pose_loader.get_pose("pre_place")
        pass_orient_pose = self.pose_loader.get_pose("pass_orient")
        pass_retreat_pose = self.pose_loader.get_pose("pass_retreat")
        post_place_pose = self.pose_loader.get_pose("post_place")
        place_pose, place_label = self.get_place_target_pose()

        # 1단계: ready 자세로 이동하고 그리퍼를 연다.
        self.wait_step("ready + gripper open")
        self.move_to_pose("ready")
        if not self.gripper.open():
            raise RuntimeError("그리퍼 열기 실패")

        # 2단계: pick 상공으로 접근한 뒤 실제 pick 위치로 하강한다.
        self.wait_step("approach pick")
        self.move_to_pose_stamped(pick_above_pose, "pick_above")
        self.move_to_pose_stamped(pick_pose, "pick")

        # 3단계: 그리퍼를 닫아 물체를 잡는다.
        self.wait_step("gripper close")
        if not self.gripper.close():
            raise RuntimeError("그리퍼 닫기 실패")

        # 4단계: pick 위치의 x/y를 유지한 채 z만 올려 들어올린다.
        self.wait_step("lift")
        self.move_to_pose_stamped(lift_pose, "lift")

        # 5단계: pass 뒤쪽 안전 위치에서 앞보기 orientation으로 바꾼다.
        self.wait_step(f"prepare {place_label} orientation")
        self.move_to_pose_stamped(place_above_pose, f"{place_label}_ready_down")
        self.move_to_pose_stamped(pass_orient_pose, f"{place_label}_ready_front")

        # 6단계: 앞보기 orientation을 유지한 채 전달 위치로 전진한다.
        self.wait_step(f"forward to {place_label}")
        self.move_to_pose_stamped(place_pose, place_label)

        # 7단계: 그리퍼를 열어 물체를 놓는다.
        self.wait_step("gripper open release")
        if not self.gripper.open():
            raise RuntimeError("물체 release를 위한 그리퍼 열기 실패")

        # 8단계: 앞보기 상태로 먼저 후퇴한 뒤 orientation을 복귀하고 ready로 간다.
        self.wait_step("retreat and return ready")
        self.move_to_pose_stamped(pass_retreat_pose, f"retreat/{place_label}_front")
        self.move_to_pose_stamped(post_place_pose, f"retreat/{place_label}_down")
        self.move_to_pose("ready")

        self.get_logger().info("=== Pick and Place 작업 완료 ===")


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
