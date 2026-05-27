"""
보완필요
Preview-oriented MoveIt client for Indy7.

기존 moveit_client.py는 그대로 두고, 여기서는 plan-only 결과를 RViz Marker로
보여준 뒤 사용자가 확인하고 실행할 수 있는 헬퍼만 추가한다.
"""

import math

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Pose, Vector3
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetCartesianPath, GetPositionFK
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from indy7_task.moveit_client import Indy7MoveItClient


class PreviewMoveItClient(Indy7MoveItClient):
    """MoveIt plan-only, FK path preview, and cautious execution helper."""

    COLOR_PLANNED = ColorRGBA(r=1.0, g=0.55, b=0.0, a=0.95)
    COLOR_ACCEPTED = ColorRGBA(r=0.1, g=0.8, b=0.25, a=0.95)
    COLOR_REJECTED = ColorRGBA(r=0.95, g=0.1, b=0.1, a=0.95)

    def __init__(self, node):
        super().__init__(node)

        self.preview_marker_topic = self._declare_and_get(
            "preview_marker_topic",
            "/indy7_task/preview_markers",
        )
        self.preview_frame_id = self._declare_and_get(
            "preview_frame_id",
            self.base_link_name,
        )
        self.preview_fk_link_name = self._declare_and_get(
            "preview_fk_link_name",
            self.end_effector_name,
        )
        self.preview_sample_limit = int(
            self._declare_and_get("preview_sample_limit", 80)
        )
        self.cartesian_service_name = self._declare_and_get(
            "cartesian_service_name",
            "compute_cartesian_path",
        )
        self.cartesian_eef_step = float(
            self._declare_and_get("cartesian_eef_step", 0.01)
        )
        self.cartesian_jump_threshold = float(
            self._declare_and_get("cartesian_jump_threshold", 0.0)
        )
        self.cartesian_revolute_jump_threshold = float(
            self._declare_and_get("cartesian_revolute_jump_threshold", 0.35)
        )
        self.cartesian_fraction_threshold = float(
            self._declare_and_get("cartesian_fraction_threshold", 0.98)
        )
        self.cartesian_avoid_collisions = self._as_bool(
            self._declare_and_get("cartesian_avoid_collisions", True)
        )
        self.cartesian_retime_joint_speed = float(
            self._declare_and_get("cartesian_retime_joint_speed", 0.15)
        )
        self.cartesian_min_segment_time = float(
            self._declare_and_get("cartesian_min_segment_time", 0.15)
        )

        qos = QoSProfile(depth=10)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.marker_pub = self.node.create_publisher(
            MarkerArray,
            self.preview_marker_topic,
            qos,
        )
        self.marker_array = MarkerArray()

        self.fk_client = self.node.create_client(
            GetPositionFK,
            "compute_fk",
            callback_group=self.callback_group,
        )
        self.cartesian_client = self.node.create_client(
            GetCartesianPath,
            self.cartesian_service_name,
            callback_group=self.callback_group,
        )

    # ----------------------------------------------------------
    #  Formatting and trajectory summary
    # ----------------------------------------------------------
    def _point_time_sec(self, point):
        duration = point.time_from_start
        return float(duration.sec) + float(duration.nanosec) * 1e-9

    def _duration_msg(self, seconds):
        whole = int(seconds)
        return Duration(
            sec=whole,
            nanosec=int((seconds - whole) * 1e9),
        )

    def _trajectory_final_joint_list(self, trajectory):
        joint_traj = trajectory.joint_trajectory
        if not joint_traj.points:
            return None

        final_point = joint_traj.points[-1]
        final_by_name = {}
        for index, joint_name in enumerate(joint_traj.joint_names):
            if index < len(final_point.positions):
                final_by_name[joint_name] = final_point.positions[index]

        if any(name not in final_by_name for name in self.joint_names):
            return None

        return [float(final_by_name[name]) for name in self.joint_names]

    def log_trajectory_summary(self, trajectory, label):
        joint_traj = trajectory.joint_trajectory
        point_count = len(joint_traj.points)
        if point_count == 0:
            self.node.get_logger().warn(f"{label}: trajectory point가 없습니다")
            return

        duration = self._point_time_sec(joint_traj.points[-1])
        final_joints = self._trajectory_final_joint_list(trajectory)
        if final_joints is None:
            final_text = "unknown"
        else:
            final_text = self._fmt_joint_degrees(final_joints)

        self.node.get_logger().info(
            f"{label}: trajectory preview "
            f"points={point_count}, duration={duration:.2f}s, "
            f"final_q_deg={final_text}"
        )

    # ----------------------------------------------------------
    #  FK preview marker
    # ----------------------------------------------------------
    def _sample_trajectory_points(self, points):
        if not points:
            return []
        limit = max(2, int(self.preview_sample_limit))
        if len(points) <= limit:
            return list(points)

        stride = max(1, len(points) // limit)
        sampled = list(points[::stride])
        if sampled[-1] is not points[-1]:
            sampled.append(points[-1])
        return sampled

    def compute_trajectory_ee_path(self, trajectory, timeout_sec=5.0):
        """RobotTrajectory 각 sampled joint point를 FK로 TCP path 점들로 바꾼다."""
        if not self.fk_client.wait_for_service(timeout_sec=timeout_sec):
            self.node.get_logger().warn(
                "compute_fk 서비스를 사용할 수 없어 trajectory preview를 생략합니다"
            )
            return []

        joint_traj = trajectory.joint_trajectory
        ee_points = []
        for point in self._sample_trajectory_points(joint_traj.points):
            if len(point.positions) < len(joint_traj.joint_names):
                continue

            request = GetPositionFK.Request()
            request.header.frame_id = self.preview_frame_id
            request.fk_link_names = [self.preview_fk_link_name]

            robot_state = RobotState()
            robot_state.joint_state = JointState()
            robot_state.joint_state.name = list(joint_traj.joint_names)
            robot_state.joint_state.position = list(point.positions)
            request.robot_state = robot_state

            response = self._wait_future(
                self.fk_client.call_async(request),
                timeout_sec,
                "compute_fk 응답",
            )
            if response is None:
                continue
            if response.error_code.val != MoveItErrorCodes.SUCCESS:
                continue
            if not response.pose_stamped:
                continue

            position = response.pose_stamped[0].pose.position
            ee_points.append((position.x, position.y, position.z))

        return ee_points

    def publish_ee_path_marker(self, ee_points, label, color=None):
        """현재 preview segment의 TCP path를 RViz LINE_STRIP으로 표시한다."""
        if not ee_points:
            return

        stamp = self.node.get_clock().now().to_msg()
        color = color or self.COLOR_PLANNED

        line = Marker()
        line.header.frame_id = self.preview_frame_id
        line.header.stamp = stamp
        line.ns = "preview_ee_path"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.006
        line.color = color
        line.points = [Point(x=x, y=y, z=z) for x, y, z in ee_points]

        text = Marker()
        text.header.frame_id = self.preview_frame_id
        text.header.stamp = stamp
        text.ns = "preview_ee_path"
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = ee_points[-1][0]
        text.pose.position.y = ee_points[-1][1]
        text.pose.position.z = ee_points[-1][2] + 0.04
        text.pose.orientation.w = 1.0
        text.scale = Vector3(x=0.0, y=0.0, z=0.035)
        text.color = color
        text.text = label

        self.marker_array.markers = [line, text]
        self.marker_pub.publish(self.marker_array)

    def preview_trajectory(self, trajectory, label, color=None):
        self.log_trajectory_summary(trajectory, label)
        ee_points = self.compute_trajectory_ee_path(trajectory)
        if ee_points:
            self.node.get_logger().info(
                f"{label}: TCP preview path {len(ee_points)}개 점 publish "
                f"({self.preview_marker_topic})"
            )
            self.publish_ee_path_marker(ee_points, label, color=color)
        return ee_points

    # ----------------------------------------------------------
    #  Plan-only helpers
    # ----------------------------------------------------------
    def plan_pose_for_preview(
        self,
        pose_stamped,
        label="pose",
        allow_pose_fallback=False,
        path_constraints=None,
    ):
        """Seeded IK joint goal을 우선 사용하고, fallback은 명시된 경우만 허용."""
        candidates = self.rank_seeded_ik_candidates(pose_stamped)
        if candidates:
            success, trajectory = self._plan_safe_joint_candidate(
                candidates,
                label,
                path_constraints=path_constraints,
            )
            if success:
                return True, trajectory

        if not allow_pose_fallback:
            self.node.get_logger().error(
                f"{label}: 안전한 IK joint-goal plan이 없어 중단합니다"
            )
            return False, None

        self.node.get_logger().warn(
            f"{label}: pose goal fallback planning을 시도합니다"
        )
        success, trajectory = self.plan_to_pose_goal(
            pose_stamped,
            path_constraints=path_constraints,
        )
        if not success or trajectory is None:
            return False, None
        if not self.check_trajectory_safety(trajectory, label=label):
            return False, None
        return True, trajectory

    def plan_joint_values_for_preview(self, joint_values, label="joint"):
        success, trajectory = self.plan_to_joint_goal(joint_values)
        if not success or trajectory is None:
            return False, None
        if not self.check_trajectory_safety(trajectory, label=label):
            return False, None
        return True, trajectory

    # ----------------------------------------------------------
    #  Cartesian path helpers
    # ----------------------------------------------------------
    def _trajectory_has_increasing_time(self, trajectory):
        previous = -math.inf
        for point in trajectory.joint_trajectory.points:
            current = self._point_time_sec(point)
            if current <= previous:
                return False
            previous = current
        return True

    def retime_trajectory_conservative(self, trajectory):
        """Cartesian service 결과에 시간이 없으면 보수적인 time_from_start를 넣는다."""
        joint_traj = trajectory.joint_trajectory
        if not joint_traj.points:
            return trajectory
        if self._trajectory_has_increasing_time(trajectory):
            return trajectory

        joint_pairs = self._trajectory_joint_pairs(trajectory)
        if not joint_pairs:
            return trajectory

        current_values = self.get_current_joint_values()
        previous = {
            joint_name: float(current_values.get(joint_name, 0.0))
            for _, joint_name in joint_pairs
        }

        speed = max(float(self.cartesian_retime_joint_speed), 0.01)
        min_segment_time = max(float(self.cartesian_min_segment_time), 0.02)
        elapsed = 0.0

        for point in joint_traj.points:
            max_delta = 0.0
            for joint_index, joint_name in joint_pairs:
                if joint_index >= len(point.positions):
                    continue
                position = float(point.positions[joint_index])
                max_delta = max(max_delta, abs(position - previous[joint_name]))
                previous[joint_name] = position

            elapsed += max(min_segment_time, max_delta / speed)
            point.time_from_start = self._duration_msg(elapsed)

        return trajectory

    def plan_cartesian_poses_for_preview(self, poses, label="cartesian"):
        """현재 state에서 주어진 Pose 리스트를 TCP Cartesian path로 계획한다."""
        if not poses:
            self.node.get_logger().error(f"{label}: cartesian waypoint가 없습니다")
            return False, None

        if not self.cartesian_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error(
                f"{self.cartesian_service_name} 서비스를 사용할 수 없습니다"
            )
            return False, None

        request = GetCartesianPath.Request()
        request.header.frame_id = self.base_link_name
        request.start_state = self.get_current_robot_state()
        request.group_name = self.group_name
        request.link_name = self.end_effector_name
        request.waypoints = [Pose(position=pose.position, orientation=pose.orientation)
                             for pose in poses]
        request.max_step = max(float(self.cartesian_eef_step), 0.001)
        request.jump_threshold = max(float(self.cartesian_jump_threshold), 0.0)
        request.prismatic_jump_threshold = 0.0
        request.revolute_jump_threshold = max(
            float(self.cartesian_revolute_jump_threshold),
            0.0,
        )
        request.avoid_collisions = bool(self.cartesian_avoid_collisions)

        response = self._wait_future(
            self.cartesian_client.call_async(request),
            self.planning_time + 10.0,
            "compute_cartesian_path 응답",
        )
        if response is None:
            return False, None

        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.node.get_logger().error(
                f"{label}: cartesian path 실패 "
                f"error_code={response.error_code.val}, "
                f"fraction={response.fraction:.3f}"
            )
            return False, None

        if response.fraction < self.cartesian_fraction_threshold:
            self.node.get_logger().error(
                f"{label}: cartesian path fraction 부족 "
                f"{response.fraction:.3f} < {self.cartesian_fraction_threshold:.3f}"
            )
            return False, None

        trajectory = self.retime_trajectory_conservative(response.solution)
        if not self.check_trajectory_safety(trajectory, label=label):
            return False, None

        return True, trajectory
