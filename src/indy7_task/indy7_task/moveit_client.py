"""
Indy7 MoveIt Action/Service Client
==================================
MoveIt Python wrapper 없이 rclpy와 moveit_msgs만 사용해 Indy7 팔을 제어한다.

주요 흐름:
  1. /joint_states를 구독해 현재 관절 상태를 저장
  2. compute_ik 서비스에 현재 RobotState를 seed로 넣어 IK 풀이
  3. IK 결과를 joint goal로 plan_only 요청
  4. execute_trajectory 액션으로 계획된 trajectory 실행
  5. IK 실패 시 pose goal planning으로 fallback
  6. 필요 시 MotionPlanRequest.path_constraints로 TCP orientation 유지

실행 방법:
  이 파일은 직접 실행하지 않는다.
  task_node.py 또는 task_node_servo.py에서 Indy7MoveItClient로 사용한다.

필요한 서버:
  /move_action
  /execute_trajectory
  /compute_ik
  /joint_states
"""

import copy
import math
import time

from geometry_msgs.msg import Pose, Quaternion, Vector3
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive


class Indy7MoveItClient:
    """MoveGroup, ExecuteTrajectory, compute_ik를 감싼 Indy7용 클라이언트."""

    SAFE_MIN_DEG = [-175.0, -175.0, -175.0, -175.0, -175.0, -215.0]
    SAFE_MAX_DEG = [175.0, 175.0, 175.0, 175.0, 175.0, 215.0]
    LIMIT_MARGIN_DEG = [10.0, 10.0, 10.0, 10.0, 10.0, 15.0]
    COST_WEIGHTS = [1.0, 1.2, 1.5, 1.0, 1.3, 0.8]
    MULTI_SEED_OFFSETS_DEG = [
        (2, 30.0),
        (2, -30.0),
        (4, 45.0),
        (4, -45.0),
    ]
    DUPLICATE_EPS_RAD = math.radians(2.0)

    def __init__(self, node):
        self.node = node
        self.callback_group = ReentrantCallbackGroup()

        # ------------------------------------------------------
        #  로봇 / 플래닝 파라미터
        # ------------------------------------------------------
        self.group_name = self._declare_and_get(
            "group_name",
            "indy_manipulator",
        )
        self.base_link_name = self._declare_and_get("base_link_name", "link0")
        self.end_effector_name = self._declare_and_get(
            "end_effector_name",
            "tcp",
        )
        self.joint_names = self._declare_and_get(
            "joint_names",
            ["joint0", "joint1", "joint2", "joint3", "joint4", "joint5"],
        )
        self.planning_time = float(
            self._declare_and_get("planning_time", 5.0)
        )
        self.num_planning_attempts = int(
            self._declare_and_get("num_planning_attempts", 5)
        )
        self.max_velocity = float(
            self._declare_and_get("max_velocity", 0.05)
        )
        self.max_acceleration = float(
            self._declare_and_get("max_acceleration", 0.05)
        )
        self.position_tolerance = float(
            self._declare_and_get("position_tolerance", 0.01)
        )
        self.orientation_tolerance = float(
            self._declare_and_get("orientation_tolerance", 0.01)
        )
        self.constrained_planning_time = float(
            self._declare_and_get("constrained_planning_time", 15.0)
        )
        self.constrained_num_planning_attempts = int(
            self._declare_and_get("constrained_num_planning_attempts", 10)
        )
        self.path_orientation_tolerance = float(
            self._declare_and_get("path_orientation_tolerance", 0.3)
        )
        self.path_orientation_z_tolerance = float(
            self._declare_and_get("path_orientation_z_tolerance", 3.14)
        )
        self.enable_multi_seed_ik = self._as_bool(
            self._declare_and_get("enable_multi_seed_ik", True)
        )
        self.multi_seed_accept_cost = float(
            self._declare_and_get("multi_seed_accept_cost", 2.5)
        )
        self.multi_seed_ik_timeout_sec = float(
            self._declare_and_get("multi_seed_ik_timeout_sec", 1.0)
        )
        self.enable_trajectory_safety = self._as_bool(
            self._declare_and_get("enable_trajectory_safety", True)
        )
        self.max_joint_delta_deg = float(
            self._declare_and_get("max_joint_delta_deg", 170.0)
        )
        self.max_waypoint_jump_deg = float(
            self._declare_and_get("max_waypoint_jump_deg", 90.0)
        )
        self.max_joint_total_motion_deg = float(
            self._declare_and_get("max_joint_total_motion_deg", 260.0)
        )
        self.max_trajectory_candidates = int(
            self._declare_and_get("max_trajectory_candidates", 3)
        )

        # ------------------------------------------------------
        #  MoveIt 액션/서비스 클라이언트와 현재 관절 상태
        # ------------------------------------------------------
        self.move_client = ActionClient(
            self.node,
            MoveGroup,
            "move_action",
            callback_group=self.callback_group,
        )
        self.execute_client = ActionClient(
            self.node,
            ExecuteTrajectory,
            "execute_trajectory",
            callback_group=self.callback_group,
        )
        self.ik_client = self.node.create_client(
            GetPositionIK,
            "compute_ik",
            callback_group=self.callback_group,
        )
        self.joint_state = None
        self.joint_state_sub = self.node.create_subscription(
            JointState,
            "joint_states",
            self._joint_state_cb,
            10,
            callback_group=self.callback_group,
        )

    # ----------------------------------------------------------
    #  파라미터와 상태 콜백
    # ----------------------------------------------------------
    def _declare_and_get(self, name, default_value):
        if not self.node.has_parameter(name):
            self.node.declare_parameter(name, default_value)
        return self.node.get_parameter(name).value

    def _as_bool(self, value):
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    def _joint_state_cb(self, msg):
        self.joint_state = msg

    # ----------------------------------------------------------
    #  서버 준비 확인
    # ----------------------------------------------------------
    def wait_for_server(self, timeout_sec=10.0):
        self.node.get_logger().info("MoveGroup 액션 서버 대기 중...")
        if not self.move_client.wait_for_server(timeout_sec=timeout_sec):
            self.node.get_logger().error(
                "MoveGroup 액션 서버를 사용할 수 없습니다"
            )
            return False
        self.node.get_logger().info("MoveGroup 액션 서버 연결 완료")
        return True

    def wait_for_servers(self, timeout_sec=10.0):
        if not self.wait_for_server(timeout_sec=timeout_sec):
            return False
        if not self.execute_client.wait_for_server(timeout_sec=timeout_sec):
            self.node.get_logger().error(
                "ExecuteTrajectory 액션 서버를 사용할 수 없습니다"
            )
            return False
        if not self.ik_client.wait_for_service(timeout_sec=timeout_sec):
            self.node.get_logger().warn(
                "compute_ik 서비스를 사용할 수 없습니다. "
                "pose goal planning fallback만 사용합니다"
            )
        return True

    def wait_for_joint_state(self, timeout_sec=10.0):
        start_time = time.monotonic()
        while self.joint_state is None:
            if time.monotonic() - start_time > timeout_sec:
                self.node.get_logger().error(
                    "joint_states 대기 시간이 초과되었습니다"
                )
                return False
            time.sleep(0.02)
        return True

    def get_current_robot_state(self):
        """현재 joint_states를 MoveIt RobotState 메시지로 변환한다."""
        robot_state = RobotState()
        if self.joint_state is not None:
            robot_state.joint_state = self.joint_state
        return robot_state

    def get_current_joint_values(self):
        """현재 Indy7 arm joint만 {joint_name: position} 형태로 반환한다."""
        if self.joint_state is None:
            return {}

        joint_values = {}
        names = self.joint_state.name
        positions = self.joint_state.position
        for index, name in enumerate(names):
            if name in self.joint_names and index < len(positions):
                joint_values[name] = positions[index]
        return joint_values

    def get_current_joint_list(self):
        """현재 Indy7 arm joint를 self.joint_names 순서의 list로 반환한다."""
        joint_values = self.get_current_joint_values()
        if any(name not in joint_values for name in self.joint_names):
            return None
        return [float(joint_values[name]) for name in self.joint_names]

    # ----------------------------------------------------------
    #  MoveGroup 요청 생성
    # ----------------------------------------------------------
    def _build_request(self, frame_id):
        """공통 MotionPlanRequest 필드를 만든다."""
        request = MotionPlanRequest()
        request.group_name = self.group_name
        request.num_planning_attempts = self.num_planning_attempts
        request.allowed_planning_time = self.planning_time
        request.max_velocity_scaling_factor = self.max_velocity
        request.max_acceleration_scaling_factor = self.max_acceleration
        if self.joint_state is not None:
            request.start_state = self.get_current_robot_state()

        request.workspace_parameters.header.frame_id = frame_id
        request.workspace_parameters.min_corner.x = -1.0
        request.workspace_parameters.min_corner.y = -1.0
        request.workspace_parameters.min_corner.z = -1.0
        request.workspace_parameters.max_corner.x = 1.0
        request.workspace_parameters.max_corner.y = 1.0
        request.workspace_parameters.max_corner.z = 1.0
        return request

    def _make_pose_constraints(self, pose, frame_id):
        """목표 TCP pose를 position/orientation constraint로 변환한다."""
        constraints = Constraints()

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = frame_id
        position_constraint.link_name = self.end_effector_name
        position_constraint.target_point_offset = Vector3()

        bounding_volume = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self.position_tolerance]
        bounding_volume.primitives.append(sphere)

        sphere_pose = Pose()
        sphere_pose.position = copy.deepcopy(pose.position)
        sphere_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        bounding_volume.primitive_poses.append(sphere_pose)

        position_constraint.constraint_region = bounding_volume
        position_constraint.weight = 1.0
        constraints.position_constraints.append(position_constraint)

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = frame_id
        orientation_constraint.link_name = self.end_effector_name
        orientation_constraint.orientation = copy.deepcopy(pose.orientation)
        orientation_constraint.absolute_x_axis_tolerance = (
            self.orientation_tolerance
        )
        orientation_constraint.absolute_y_axis_tolerance = (
            self.orientation_tolerance
        )
        orientation_constraint.absolute_z_axis_tolerance = (
            self.orientation_tolerance
        )
        orientation_constraint.weight = 1.0
        constraints.orientation_constraints.append(orientation_constraint)

        return constraints

    def make_orientation_path_constraint(
        self,
        pose_stamped,
        x_tolerance=None,
        y_tolerance=None,
        z_tolerance=None,
    ):
        """경로 전체에서 유지할 TCP orientation path constraint를 만든다.

        예제 ex08_constraints.py의 핵심 패턴을 Indy7용으로 옮긴 함수다.
        기본값은 물체를 기울이지 않는 데 초점을 맞춰 x/y 회전은 제한하고,
        z축 yaw는 비교적 자유롭게 둔다. TCP yaw까지 고정해야 하면
        z_tolerance를 0.1~0.3 정도로 명시해서 호출하면 된다.
        """
        frame_id = pose_stamped.header.frame_id or self.base_link_name
        x_tol = (
            self.path_orientation_tolerance
            if x_tolerance is None
            else float(x_tolerance)
        )
        y_tol = (
            self.path_orientation_tolerance
            if y_tolerance is None
            else float(y_tolerance)
        )
        z_tol = (
            self.path_orientation_z_tolerance
            if z_tolerance is None
            else float(z_tolerance)
        )

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = frame_id
        orientation_constraint.link_name = self.end_effector_name
        orientation_constraint.orientation = copy.deepcopy(
            pose_stamped.pose.orientation
        )
        orientation_constraint.absolute_x_axis_tolerance = x_tol
        orientation_constraint.absolute_y_axis_tolerance = y_tol
        orientation_constraint.absolute_z_axis_tolerance = z_tol
        orientation_constraint.weight = 1.0

        constraints = Constraints()
        constraints.orientation_constraints.append(orientation_constraint)
        return constraints

    def _apply_path_constraints(self, request, path_constraints):
        """MotionPlanRequest에 path_constraints와 보수적 planning 값을 적용한다."""
        if path_constraints is None:
            return

        request.path_constraints = path_constraints
        # 경로 제약은 일반 planning보다 실패 확률이 높아서 예제처럼
        # planning 시간/시도 횟수를 올린다. 기존 launch 값이 더 크면 유지한다.
        request.allowed_planning_time = max(
            request.allowed_planning_time,
            self.constrained_planning_time,
        )
        request.num_planning_attempts = max(
            request.num_planning_attempts,
            self.constrained_num_planning_attempts,
        )

    # ----------------------------------------------------------
    #  액션/서비스 대기 헬퍼
    # ----------------------------------------------------------
    def _wait_future(self, future, timeout_sec, label):
        start_time = time.monotonic()
        while not future.done():
            if time.monotonic() - start_time > timeout_sec:
                self.node.get_logger().error(f"{label} 대기 시간이 초과되었습니다")
                return None
            time.sleep(0.02)
        return future.result()

    def _send_request(self, request, plan_only=False):
        """MoveGroup action에 planning request를 보내고 결과를 기다린다."""
        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = plan_only
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3
        planning_timeout = max(
            float(request.allowed_planning_time),
            self.planning_time,
        )

        goal_handle = self._wait_future(
            self.move_client.send_goal_async(goal),
            planning_timeout + 5.0,
            "MoveGroup 목표 응답",
        )

        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error("MoveGroup 목표가 거부되었습니다")
            return False, None

        result_wrapper = self._wait_future(
            goal_handle.get_result_async(),
            planning_timeout + 30.0,
            "MoveGroup 결과",
        )
        if result_wrapper is None:
            return False, None

        result = result_wrapper.result

        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            return True, result.planned_trajectory

        self.node.get_logger().error(
            f"MoveGroup 실패: error_code={result.error_code.val}"
        )
        return False, None

    def _make_joint_constraints(self, joint_values):
        """IK 결과 joint dictionary를 JointConstraint 묶음으로 변환한다."""
        constraints = Constraints()
        for joint_name, value in joint_values.items():
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = float(value)
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        return constraints

    # ----------------------------------------------------------
    #  플래닝 / 실행 기본 함수
    # ----------------------------------------------------------
    def plan_to_joint_goal(self, joint_values, path_constraints=None):
        """joint goal로 계획만 수행하고 trajectory를 반환한다.

        path_constraints가 주어지면 IK로 얻은 joint goal까지 가는 동안에도
        TCP orientation 같은 경로 제약을 계속 만족하도록 요청한다.
        """
        request = self._build_request(self.base_link_name)
        self._apply_path_constraints(request, path_constraints)
        request.goal_constraints.append(
            self._make_joint_constraints(joint_values)
        )
        return self._send_request(request, plan_only=True)

    def plan_to_pose_goal(self, pose_stamped, path_constraints=None):
        """pose goal로 계획만 수행하고 trajectory를 반환한다."""
        frame_id = pose_stamped.header.frame_id or self.base_link_name
        request = self._build_request(frame_id)
        self._apply_path_constraints(request, path_constraints)
        request.goal_constraints.append(
            self._make_pose_constraints(pose_stamped.pose, frame_id)
        )
        return self._send_request(request, plan_only=True)

    def plan_to_pose_goal_with_constraints(
        self,
        pose_stamped,
        path_constraints,
    ):
        """예제 utils.py와 같은 이름의 경로 제약 pose planning 헬퍼."""
        return self.plan_to_pose_goal(
            pose_stamped,
            path_constraints=path_constraints,
        )

    def execute_trajectory(self, trajectory):
        """MoveGroup이 만든 trajectory를 ExecuteTrajectory action으로 실행한다."""
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        goal_handle = self._wait_future(
            self.execute_client.send_goal_async(goal),
            5.0,
            "ExecuteTrajectory 목표 응답",
        )
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error(
                "ExecuteTrajectory 목표가 거부되었습니다"
            )
            return False

        result_wrapper = self._wait_future(
            goal_handle.get_result_async(),
            self.planning_time + 30.0,
            "ExecuteTrajectory 결과",
        )
        if result_wrapper is None:
            return False

        result = result_wrapper.result
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            return True

        self.node.get_logger().error(
            "ExecuteTrajectory 실패: "
            f"error_code={result.error_code.val}"
        )
        return False

    # ----------------------------------------------------------
    #  궤적 안전 검사
    # ----------------------------------------------------------
    def _trajectory_joint_pairs(self, trajectory):
        """trajectory 안에서 Indy7 arm joint의 index와 이름만 뽑는다."""
        joint_names = trajectory.joint_trajectory.joint_names
        return [
            (index, name)
            for index, name in enumerate(joint_names)
            if name in self.joint_names
        ]

    def _max_allowed_rad(self, value_deg):
        """0 이하 값이면 해당 안전 기준을 끈 것으로 본다."""
        if value_deg <= 0.0:
            return math.inf
        return math.radians(value_deg)

    def _log_trajectory_safety_error(
        self,
        label,
        joint_name,
        reason,
        actual_rad,
        limit_deg,
        point_index,
    ):
        self.node.get_logger().error(
            f"{label}: 궤적 안전 검사 실패 - "
            f"{joint_name} {reason} "
            f"{math.degrees(actual_rad):.1f}deg > {limit_deg:.1f}deg "
            f"(point {point_index})"
        )

    def check_trajectory_safety(self, trajectory, label="trajectory"):
        """실행 전에 관절이 과하게 도는 궤적인지 검사한다.

        multi-seed IK는 목표 joint 해가 현재 자세에서 가까운지를 고른다.
        이 함수는 그 다음 단계로, MoveIt이 실제로 만든 trajectory 내부에
        큰 관절 점프나 한 바퀴에 가까운 우회 동작이 있는지 확인한다.
        """
        if not self.enable_trajectory_safety:
            return True

        joint_trajectory = trajectory.joint_trajectory
        if not joint_trajectory.points:
            self.node.get_logger().error(
                f"{label}: trajectory point가 비어 있어 실행하지 않습니다"
            )
            return False

        joint_pairs = self._trajectory_joint_pairs(trajectory)
        if not joint_pairs:
            self.node.get_logger().error(
                f"{label}: trajectory에 Indy7 arm joint가 없어 실행하지 않습니다"
            )
            return False

        current_values = self.get_current_joint_values()
        first_point = joint_trajectory.points[0]
        start_positions = {}
        previous_positions = {}
        total_motion = {}

        for joint_index, joint_name in joint_pairs:
            if joint_index >= len(first_point.positions):
                self.node.get_logger().error(
                    f"{label}: trajectory 첫 point에 {joint_name} 위치가 없습니다"
                )
                return False

            first_position = float(first_point.positions[joint_index])
            start_position = float(current_values.get(joint_name, first_position))
            start_positions[joint_name] = start_position
            previous_positions[joint_name] = start_position
            total_motion[joint_name] = 0.0

        max_delta_rad = self._max_allowed_rad(self.max_joint_delta_deg)
        max_jump_rad = self._max_allowed_rad(self.max_waypoint_jump_deg)
        max_total_rad = self._max_allowed_rad(self.max_joint_total_motion_deg)

        for point_index, point in enumerate(joint_trajectory.points):
            for joint_index, joint_name in joint_pairs:
                if joint_index >= len(point.positions):
                    self.node.get_logger().error(
                        f"{label}: trajectory point {point_index}에 "
                        f"{joint_name} 위치가 없습니다"
                    )
                    return False

                position = float(point.positions[joint_index])
                delta_from_start = abs(position - start_positions[joint_name])
                waypoint_jump = abs(position - previous_positions[joint_name])
                total_motion[joint_name] += waypoint_jump

                if delta_from_start > max_delta_rad:
                    self._log_trajectory_safety_error(
                        label,
                        joint_name,
                        "시작점 대비 변화량",
                        delta_from_start,
                        self.max_joint_delta_deg,
                        point_index,
                    )
                    return False

                if waypoint_jump > max_jump_rad:
                    self._log_trajectory_safety_error(
                        label,
                        joint_name,
                        "waypoint 점프",
                        waypoint_jump,
                        self.max_waypoint_jump_deg,
                        point_index,
                    )
                    return False

                if total_motion[joint_name] > max_total_rad:
                    self._log_trajectory_safety_error(
                        label,
                        joint_name,
                        "누적 이동량",
                        total_motion[joint_name],
                        self.max_joint_total_motion_deg,
                        point_index,
                    )
                    return False

                previous_positions[joint_name] = position

        return True

    # ----------------------------------------------------------
    #  현재 관절 seed IK + 부드러운 이동
    # ----------------------------------------------------------
    def _joint_list_to_dict(self, joints):
        return {
            joint_name: float(joints[index])
            for index, joint_name in enumerate(self.joint_names)
        }

    def _joint_dict_to_list(self, joint_values):
        if any(name not in joint_values for name in self.joint_names):
            return None
        return [float(joint_values[name]) for name in self.joint_names]

    def _limit_min_rad(self, index):
        if index >= len(self.SAFE_MIN_DEG):
            return -math.inf
        return math.radians(self.SAFE_MIN_DEG[index])

    def _limit_max_rad(self, index):
        if index >= len(self.SAFE_MAX_DEG):
            return math.inf
        return math.radians(self.SAFE_MAX_DEG[index])

    def _limit_margin_rad(self, index):
        if index >= len(self.LIMIT_MARGIN_DEG):
            return 0.0
        return math.radians(self.LIMIT_MARGIN_DEG[index])

    def _within_safe_bounds(self, joints):
        for index, value in enumerate(joints):
            if value < self._limit_min_rad(index):
                return False
            if value > self._limit_max_rad(index):
                return False
        return True

    def _within_limit_margin(self, joints):
        for index, value in enumerate(joints):
            margin = self._limit_margin_rad(index)
            if value - self._limit_min_rad(index) < margin:
                return False
            if self._limit_max_rad(index) - value < margin:
                return False
        return True

    def _clip_to_safe_bounds(self, joints):
        clipped = []
        for index, value in enumerate(joints):
            clipped.append(
                min(
                    max(float(value), self._limit_min_rad(index)),
                    self._limit_max_rad(index),
                )
            )
        return clipped

    def _same_joint_solution(self, first, second):
        return all(
            abs(a - b) < self.DUPLICATE_EPS_RAD
            for a, b in zip(first, second)
        )

    def _joint_delta_cost(self, current_joints, goal_joints):
        cost = 0.0
        for index, (current, goal) in enumerate(
            zip(current_joints, goal_joints)
        ):
            weight = (
                self.COST_WEIGHTS[index]
                if index < len(self.COST_WEIGHTS)
                else 1.0
            )
            cost += weight * abs(goal - current)
        return cost

    def _fmt_joint_degrees(self, joints):
        return str([round(math.degrees(value), 2) for value in joints])

    def _make_seed_robot_state(self, seed_joints):
        robot_state = RobotState()
        robot_state.joint_state = JointState()
        robot_state.joint_state.name = list(self.joint_names)
        robot_state.joint_state.position = [float(value) for value in seed_joints]
        return robot_state

    def _extract_ik_joint_values(self, response):
        joint_values = {}
        names = response.solution.joint_state.name
        positions = response.solution.joint_state.position
        for index, name in enumerate(names):
            if name in self.joint_names and index < len(positions):
                joint_values[name] = positions[index]

        missing = [
            name for name in self.joint_names
            if name not in joint_values
        ]
        if missing:
            self.node.get_logger().warn(
                "현재 관절 seed IK 결과에 필요한 joint가 없습니다: "
                f"{', '.join(missing)}"
            )
            return None
        return joint_values

    def _call_seeded_ik_once(
        self,
        pose_stamped,
        seed_joints,
        avoid_collisions=False,
        seed_label="seed",
    ):
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.group_name
        request.ik_request.ik_link_name = self.end_effector_name
        request.ik_request.robot_state = self._make_seed_robot_state(seed_joints)
        request.ik_request.pose_stamped = pose_stamped
        request.ik_request.avoid_collisions = bool(avoid_collisions)

        ik_timeout = max(float(self.multi_seed_ik_timeout_sec), 0.1)
        request.ik_request.timeout.sec = int(ik_timeout)
        request.ik_request.timeout.nanosec = int(
            (ik_timeout - int(ik_timeout)) * 1e9
        )

        response = self._wait_future(
            self.ik_client.call_async(request),
            max(ik_timeout + 1.0, 2.0),
            f"compute_ik 응답({seed_label})",
        )
        if response is None:
            return None
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            message = f"{seed_label} IK 실패: error_code={response.error_code.val}"
            if seed_label == "current":
                self.node.get_logger().warn(message)
            else:
                self.node.get_logger().debug(message)
            return None

        return self._extract_ik_joint_values(response)

    def _generate_multi_seed_candidates(self, current_joints):
        seeds = []
        for joint_index, offset_deg in self.MULTI_SEED_OFFSETS_DEG:
            if joint_index >= len(current_joints):
                continue
            seed = list(current_joints)
            seed[joint_index] += math.radians(offset_deg)
            seed = self._clip_to_safe_bounds(seed)
            if self._same_joint_solution(seed, current_joints):
                continue
            if any(self._same_joint_solution(seed, existing) for existing in seeds):
                continue
            seeds.append(seed)
        return seeds

    def rank_seeded_ik_candidates(self, pose_stamped, avoid_collisions=False):
        """현재 관절 기준 IK 후보를 가까운 순서대로 만든다.

        여기서는 IK 해의 joint cost까지만 보고 정렬한다. 실제 trajectory가
        안전한지는 go_smooth()에서 후보별 planning 후 다시 검사한다.
        """
        if not self.ik_client.service_is_ready():
            return []

        current_joints = self.get_current_joint_list()
        if current_joints is None:
            self.node.get_logger().warn(
                "현재 joint_states에 필요한 arm joint가 없어 IK seed를 만들 수 없습니다"
            )
            return []

        first_solution = self._call_seeded_ik_once(
            pose_stamped,
            current_joints,
            avoid_collisions=avoid_collisions,
            seed_label="current",
        )
        first_list = (
            self._joint_dict_to_list(first_solution)
            if first_solution is not None
            else None
        )
        candidates = []
        fallback_candidate = None

        if not self.enable_multi_seed_ik:
            if first_solution is None:
                self.node.get_logger().warn("현재 관절 seed IK 실패")
                return []
            first_cost = self._joint_delta_cost(current_joints, first_list)
            return [(first_cost, first_solution, first_list, "current")]

        if first_list is not None:
            first_cost = self._joint_delta_cost(current_joints, first_list)
            if (
                self._within_safe_bounds(first_list)
                and self._within_limit_margin(first_list)
                and first_cost <= self.multi_seed_accept_cost
            ):
                return [(first_cost, first_solution, first_list, "current")]

            self.node.get_logger().debug(
                "현재 seed IK가 멀거나 limit margin에 가까워 "
                "추가 seed를 탐색합니다: "
                f"cost={first_cost:.3f}, q={self._fmt_joint_degrees(first_list)}"
            )
            if self._within_safe_bounds(first_list):
                if self._within_limit_margin(first_list):
                    candidates.append(
                        (first_cost, first_solution, first_list, "current")
                    )
                else:
                    fallback_candidate = (
                        first_cost,
                        first_solution,
                        first_list,
                        "current-fallback",
                    )
                    self.node.get_logger().warn(
                        "현재 seed IK 해가 limit margin에 가까워 "
                        "ranking 후보에서 제외합니다"
                    )
            else:
                self.node.get_logger().warn(
                    "현재 seed IK 해가 hard limit 밖이라 후보에서 제외합니다"
                )
        else:
            self.node.get_logger().warn(
                "현재 관절 seed IK 실패. 추가 seed를 탐색합니다"
            )

        for index, seed in enumerate(
            self._generate_multi_seed_candidates(current_joints),
            start=1,
        ):
            joint_values = self._call_seeded_ik_once(
                pose_stamped,
                seed,
                avoid_collisions=avoid_collisions,
                seed_label=f"multi-{index}",
            )
            if joint_values is None:
                continue

            joint_list = self._joint_dict_to_list(joint_values)
            if joint_list is None:
                continue
            if not self._within_safe_bounds(joint_list):
                self.node.get_logger().warn(
                    f"[multi-{index}] IK 해가 hard limit 밖이라 제외: "
                    f"{self._fmt_joint_degrees(joint_list)}"
                )
                continue
            if not self._within_limit_margin(joint_list):
                self.node.get_logger().warn(
                    f"[multi-{index}] IK 해가 limit margin에 가까워 제외: "
                    f"{self._fmt_joint_degrees(joint_list)}"
                )
                continue
            if any(
                self._same_joint_solution(joint_list, existing[2])
                for existing in candidates
            ):
                continue

            cost = self._joint_delta_cost(current_joints, joint_list)
            candidates.append((cost, joint_values, joint_list, f"multi-{index}"))

        if not candidates:
            if fallback_candidate is not None:
                self.node.get_logger().warn(
                    "추가 seed에서 더 나은 IK 해를 찾지 못해 현재 seed 해를 사용합니다"
                )
                return [fallback_candidate]
            self.node.get_logger().warn("모든 multi-seed IK 시도가 실패했습니다")
            return []

        candidates.sort(key=lambda item: item[0])
        return candidates

    def solve_seeded_ik(self, pose_stamped, avoid_collisions=False):
        """기존 호출부를 위한 단일 IK 해 반환 helper."""
        candidates = self.rank_seeded_ik_candidates(
            pose_stamped,
            avoid_collisions=avoid_collisions,
        )
        if not candidates:
            return None

        best_cost, best_values, best_list, best_label = candidates[0]
        self.node.get_logger().debug(
            "multi-seed IK 선택: "
            f"{best_label}, cost={best_cost:.3f}, "
            f"q={self._fmt_joint_degrees(best_list)}"
        )
        return best_values

    def _plan_safe_joint_candidate(
        self,
        candidates,
        label,
        path_constraints=None,
    ):
        """IK 후보별 planning + safety check로 실행할 trajectory를 고른다."""
        max_candidates = max(1, int(self.max_trajectory_candidates))
        for attempt, candidate in enumerate(
            candidates[:max_candidates],
            start=1,
        ):
            cost, joint_values, joint_list, seed_label = candidate
            self.node.get_logger().debug(
                f"{label}: trajectory 후보 {attempt}/{max_candidates} "
                f"planning - {seed_label}, cost={cost:.3f}, "
                f"q={self._fmt_joint_degrees(joint_list)}"
            )

            plan_success, trajectory = self.plan_to_joint_goal(
                joint_values,
                path_constraints=path_constraints,
            )
            if not plan_success or trajectory is None:
                self.node.get_logger().warn(
                    f"{label}: {seed_label} 후보 planning 실패"
                )
                continue

            safety_label = f"{label}/{seed_label}"
            if not self.check_trajectory_safety(
                trajectory,
                label=safety_label,
            ):
                self.node.get_logger().warn(
                    f"{label}: {seed_label} 후보는 안전검사를 통과하지 못했습니다"
                )
                continue

            self.node.get_logger().debug(
                f"{label}: trajectory 후보 선택 - {seed_label}"
            )
            return True, trajectory

        return False, None

    def go_smooth(self, pose_stamped, label="pose"):
        """현재 관절 seed IK -> joint goal planning -> trajectory 실행."""
        candidates = self.rank_seeded_ik_candidates(pose_stamped)
        if candidates:
            plan_success, trajectory = self._plan_safe_joint_candidate(
                candidates,
                label,
            )
            if not plan_success:
                self.node.get_logger().warn(
                    f"{label}: 안전한 IK trajectory 후보를 찾지 못해 "
                    "pose goal로 fallback합니다"
                )
                plan_success, trajectory = self.plan_to_pose_goal(pose_stamped)
        else:
            self.node.get_logger().warn(
                f"{label}: 현재 관절 seed IK를 사용할 수 없어 "
                "pose goal로 fallback합니다"
            )
            plan_success, trajectory = self.plan_to_pose_goal(pose_stamped)

        if not plan_success or trajectory is None:
            self.node.get_logger().error(f"{label}: planning 실패")
            return False

        if not self.check_trajectory_safety(trajectory, label=label):
            return False

        if self.execute_trajectory(trajectory):
            self.node.get_logger().info(f"{label} 이동 완료")
            return True
        return False

    def go_smooth_with_constraints(
        self,
        pose_stamped,
        path_constraints,
        label="pose",
        avoid_collisions=False,
    ):
        """현재 관절 seed IK 흐름에 path_constraints를 얹어 실행한다.

        예제의 path constraint planning과 현재 파일의 seeded IK planning을
        합친 함수다. IK가 성공하면 joint goal planning에 path_constraints를
        넣고, IK가 실패하면 pose goal fallback에도 같은 제약을 넣는다.
        """
        candidates = self.rank_seeded_ik_candidates(
            pose_stamped,
            avoid_collisions=avoid_collisions,
        )
        if candidates:
            plan_success, trajectory = self._plan_safe_joint_candidate(
                candidates,
                label,
                path_constraints=path_constraints,
            )
            if not plan_success:
                self.node.get_logger().warn(
                    f"{label}: 안전한 경로 제약 IK trajectory 후보를 "
                    "찾지 못해 pose goal로 fallback합니다"
                )
                plan_success, trajectory = self.plan_to_pose_goal(
                    pose_stamped,
                    path_constraints=path_constraints,
                )
        else:
            self.node.get_logger().warn(
                f"{label}: 현재 관절 seed IK를 사용할 수 없어 "
                "경로 제약 pose goal로 fallback합니다"
            )
            plan_success, trajectory = self.plan_to_pose_goal(
                pose_stamped,
                path_constraints=path_constraints,
            )

        if not plan_success or trajectory is None:
            self.node.get_logger().error(
                f"{label}: 경로 제약 planning 실패"
            )
            return False

        if not self.check_trajectory_safety(trajectory, label=label):
            return False

        if self.execute_trajectory(trajectory):
            self.node.get_logger().info(f"{label} 이동 완료")
            return True
        return False

    def go_smooth_with_orientation_constraint(
        self,
        pose_stamped,
        label="pose",
        reference_pose_stamped=None,
        x_tolerance=None,
        y_tolerance=None,
        z_tolerance=None,
        avoid_collisions=False,
    ):
        """기준 pose의 TCP orientation을 유지하며 목표 pose로 이동한다."""
        reference_pose = reference_pose_stamped or pose_stamped
        path_constraints = self.make_orientation_path_constraint(
            reference_pose,
            x_tolerance=x_tolerance,
            y_tolerance=y_tolerance,
            z_tolerance=z_tolerance,
        )
        return self.go_smooth_with_constraints(
            pose_stamped,
            path_constraints,
            label=label,
            avoid_collisions=avoid_collisions,
        )

    # ----------------------------------------------------------
    #  직접 pose 실행 fallback
    # ----------------------------------------------------------
    def move_to_pose(self, pose_stamped, label="pose", path_constraints=None):
        """pose constraint를 MoveGroup에 바로 보내 plan+execute를 수행한다."""
        frame_id = pose_stamped.header.frame_id or self.base_link_name
        pose = pose_stamped.pose

        self.node.get_logger().info(
            f"{label} 이동 요청: "
            f"frame={frame_id}, "
            f"ee={self.end_effector_name}, "
            f"pos=({pose.position.x:.3f}, "
            f"{pose.position.y:.3f}, "
            f"{pose.position.z:.3f})"
        )

        if not self.wait_for_servers(timeout_sec=10.0):
            return False
        if not self.wait_for_joint_state(timeout_sec=10.0):
            return False

        request = self._build_request(frame_id)
        self._apply_path_constraints(request, path_constraints)
        request.goal_constraints.append(
            self._make_pose_constraints(pose, frame_id)
        )

        success, _ = self._send_request(request, plan_only=False)
        if success:
            self.node.get_logger().info(f"{label} 이동 완료")
            return True
        return False
