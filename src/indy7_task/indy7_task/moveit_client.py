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

    def __init__(self, node):
        self.node = node
        self.callback_group = ReentrantCallbackGroup()

        # ------------------------------------------------------
        #  Robot / planning parameters
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
            self._declare_and_get("max_velocity", 0.2)
        )
        self.max_acceleration = float(
            self._declare_and_get("max_acceleration", 0.2)
        )
        self.position_tolerance = float(
            self._declare_and_get("position_tolerance", 0.01)
        )
        self.orientation_tolerance = float(
            self._declare_and_get("orientation_tolerance", 0.01)
        )

        # ------------------------------------------------------
        #  MoveIt action/service clients and current joint state
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
    #  Parameters and state callbacks
    # ----------------------------------------------------------
    def _declare_and_get(self, name, default_value):
        if not self.node.has_parameter(name):
            self.node.declare_parameter(name, default_value)
        return self.node.get_parameter(name).value

    def _joint_state_cb(self, msg):
        self.joint_state = msg

    # ----------------------------------------------------------
    #  Server readiness
    # ----------------------------------------------------------
    def wait_for_server(self, timeout_sec=10.0):
        self.node.get_logger().info("Waiting for MoveGroup action server...")
        if not self.move_client.wait_for_server(timeout_sec=timeout_sec):
            self.node.get_logger().error(
                "MoveGroup action server is unavailable"
            )
            return False
        self.node.get_logger().info("MoveGroup action server is available")
        return True

    def wait_for_servers(self, timeout_sec=10.0):
        if not self.wait_for_server(timeout_sec=timeout_sec):
            return False
        if not self.execute_client.wait_for_server(timeout_sec=timeout_sec):
            self.node.get_logger().error(
                "ExecuteTrajectory action server is unavailable"
            )
            return False
        if not self.ik_client.wait_for_service(timeout_sec=timeout_sec):
            self.node.get_logger().warn(
                "compute_ik service is unavailable; "
                "pose planning fallback only"
            )
        return True

    def wait_for_joint_state(self, timeout_sec=10.0):
        start_time = time.monotonic()
        while self.joint_state is None:
            if time.monotonic() - start_time > timeout_sec:
                self.node.get_logger().error(
                    "Timed out waiting for joint_states"
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

    # ----------------------------------------------------------
    #  MoveGroup request builders
    # ----------------------------------------------------------
    def _build_request(self, frame_id):
        """공통 MotionPlanRequest 필드를 만든다."""
        request = MotionPlanRequest()
        request.group_name = self.group_name
        request.num_planning_attempts = self.num_planning_attempts
        request.allowed_planning_time = self.planning_time
        request.max_velocity_scaling_factor = self.max_velocity
        request.max_acceleration_scaling_factor = self.max_acceleration

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

    # ----------------------------------------------------------
    #  Action/service waiting helpers
    # ----------------------------------------------------------
    def _wait_future(self, future, timeout_sec, label):
        start_time = time.monotonic()
        while not future.done():
            if time.monotonic() - start_time > timeout_sec:
                self.node.get_logger().error(f"Timed out waiting for {label}")
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

        goal_handle = self._wait_future(
            self.move_client.send_goal_async(goal),
            self.planning_time + 5.0,
            "MoveGroup goal response",
        )

        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error("MoveGroup goal was rejected")
            return False, None

        result_wrapper = self._wait_future(
            goal_handle.get_result_async(),
            self.planning_time + 30.0,
            "MoveGroup result",
        )
        if result_wrapper is None:
            return False, None

        result = result_wrapper.result

        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            return True, result.planned_trajectory

        self.node.get_logger().error(
            f"MoveGroup failed with error code {result.error_code.val}"
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
    #  Planning / execution primitives
    # ----------------------------------------------------------
    def plan_to_joint_goal(self, joint_values):
        """joint goal로 계획만 수행하고 trajectory를 반환한다."""
        request = self._build_request(self.base_link_name)
        request.goal_constraints.append(
            self._make_joint_constraints(joint_values)
        )
        return self._send_request(request, plan_only=True)

    def plan_to_pose_goal(self, pose_stamped):
        """pose goal로 계획만 수행하고 trajectory를 반환한다."""
        frame_id = pose_stamped.header.frame_id or self.base_link_name
        request = self._build_request(frame_id)
        request.goal_constraints.append(
            self._make_pose_constraints(pose_stamped.pose, frame_id)
        )
        return self._send_request(request, plan_only=True)

    def execute_trajectory(self, trajectory):
        """MoveGroup이 만든 trajectory를 ExecuteTrajectory action으로 실행한다."""
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        goal_handle = self._wait_future(
            self.execute_client.send_goal_async(goal),
            5.0,
            "ExecuteTrajectory goal response",
        )
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error("ExecuteTrajectory goal was rejected")
            return False

        result_wrapper = self._wait_future(
            goal_handle.get_result_async(),
            self.planning_time + 30.0,
            "ExecuteTrajectory result",
        )
        if result_wrapper is None:
            return False

        result = result_wrapper.result
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            return True

        self.node.get_logger().error(
            "ExecuteTrajectory failed with error code "
            f"{result.error_code.val}"
        )
        return False

    # ----------------------------------------------------------
    #  Seeded IK + smooth motion
    # ----------------------------------------------------------
    def solve_seeded_ik(self, pose_stamped):
        """현재 관절 상태를 seed로 사용해 목표 pose의 IK를 푼다."""
        if not self.ik_client.service_is_ready():
            return None

        request = GetPositionIK.Request()
        request.ik_request.group_name = self.group_name
        request.ik_request.robot_state = self.get_current_robot_state()
        request.ik_request.pose_stamped = pose_stamped
        request.ik_request.avoid_collisions = False

        response = self._wait_future(
            self.ik_client.call_async(request),
            self.planning_time,
            "compute_ik response",
        )
        if response is None:
            return None
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.node.get_logger().warn(
                f"Seeded IK failed with code {response.error_code.val}"
            )
            return None

        joint_values = {}
        names = response.solution.joint_state.name
        positions = response.solution.joint_state.position
        for index, name in enumerate(names):
            if name in self.joint_names and index < len(positions):
                joint_values[name] = positions[index]
        return joint_values

    def go_smooth(self, pose_stamped, label="pose"):
        """seeded IK -> joint goal planning -> trajectory 실행."""
        joint_values = self.solve_seeded_ik(pose_stamped)
        if joint_values:
            plan_success, trajectory = self.plan_to_joint_goal(joint_values)
        else:
            self.node.get_logger().warn(
                f"{label}: seeded IK unavailable, falling back to pose goal"
            )
            plan_success, trajectory = self.plan_to_pose_goal(pose_stamped)

        if not plan_success or trajectory is None:
            self.node.get_logger().error(f"{label}: planning failed")
            return False

        if self.execute_trajectory(trajectory):
            self.node.get_logger().info(f"Move to {label} done")
            return True
        return False

    # ----------------------------------------------------------
    #  Direct pose execution fallback
    # ----------------------------------------------------------
    def move_to_pose(self, pose_stamped, label="pose"):
        """pose constraint를 MoveGroup에 바로 보내 plan+execute를 수행한다."""
        frame_id = pose_stamped.header.frame_id or self.base_link_name
        pose = pose_stamped.pose

        self.node.get_logger().info(
            f"Move to {label}: "
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
        request.goal_constraints.append(
            self._make_pose_constraints(pose, frame_id)
        )

        success, _ = self._send_request(request, plan_only=False)
        if success:
            self.node.get_logger().info(f"Move to {label} done")
            return True
        return False
