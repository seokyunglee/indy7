"""
HRI Adaptive Indy7 Pick & Pass Task
===================================
첫 cycle은 YAML의 기본 pass 위치를 사용하고, 두 번째 cycle부터는 HRI 시스템이
새로 저장한 pass_place_goal.json이 있으면 pass 위치를 갱신한다. 새 JSON이
없으면 직전에 사용한 pass 위치를 유지한다.

기본 task_node.py는 고정 YAML pass 데모로 유지하고, 이 파일에서만 JSON 기반
pass 위치 변경을 다룬다.

실행 예시:
  ros2 run indy7_task hri_adaptive_task --ros-args \
    -p repeat_count:=30 \
    -p pass_place_goal_path:=/home/leeseo/indy_project/src/indy7_task/config/pass_place_goal.json

  # 속도/가속도 낮춰서 실행
  ros2 run indy7_task hri_adaptive_task --ros-args \
    -p repeat_count:=30 \
    -p pass_place_goal_path:=/home/leeseo/indy_project/src/indy7_task/config/pass_place_goal.json \
    -p max_velocity:=0.05 \
    -p max_acceleration:=0.05

  ros2 run indy7_task hri_adaptive_task --ros-args \
    -p max_velocity:=3.00 \
    -p max_acceleration:=3.00
"""

import json
import os
import select
import time
import termios
import tty
from threading import Thread

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from std_srvs.srv import Trigger

from indy7_task.task_node import Indy7TaskNode, as_bool


class HRIAdaptiveTaskNode(Indy7TaskNode):
    """HRI JSON pass 목표를 cycle 단위로 반영하는 실험용 노드."""

    def __init__(self):
        super().__init__()

        package_share = get_package_share_directory("indy7_task")
        default_pass_place_goal = (
            f"/home/leeseo/Cobot_2x2x2_HRI_Experiment/json_log/pass_place_goal.json"
        )

        self.declare_parameter("pass_place_goal_path", default_pass_place_goal)
        self.declare_parameter("use_yaml_pass_first_cycle", True)
        self.declare_parameter("manual_gripper_open_with_space", True)
        self.declare_parameter("pass_goal_poll_sec", 0.1)
        self.declare_parameter("pass_goal_wait_timeout_sec", 0.0)
        self.declare_parameter("repeat_count", 10)
        self.declare_parameter("cycle_wait_sec", 0.5)
        self.declare_parameter("task_phase_topic", "/indy7/task_phase")
        self.declare_parameter(
            "consume_hold_release_service",
            "/indy7/consume_hold_release",
        )
        self.declare_parameter("hold_release_poll_sec", 0.1)
        self.declare_parameter(
            "get_review_pending_service",
            "/indy7/get_review_pending",
        )
        self.declare_parameter("review_pending_poll_sec", 0.1)
        self.declare_parameter("review_pending_wait_timeout_sec", 0.0)

        self.pass_place_goal_path = self.get_parameter(
            "pass_place_goal_path"
        ).value
        self.use_yaml_pass_first_cycle = as_bool(
            self.get_parameter("use_yaml_pass_first_cycle").value
        )
        self.manual_gripper_open_with_space = as_bool(
            self.get_parameter("manual_gripper_open_with_space").value
        )
        self.pass_goal_poll_sec = float(
            self.get_parameter("pass_goal_poll_sec").value
        )
        self.pass_goal_wait_timeout_sec = float(
            self.get_parameter("pass_goal_wait_timeout_sec").value
        )
        self.repeat_count = int(self.get_parameter("repeat_count").value)
        self.cycle_wait_sec = float(
            self.get_parameter("cycle_wait_sec").value
        )
        self.task_phase_topic = self.get_parameter("task_phase_topic").value
        self.consume_hold_release_service = self.get_parameter(
            "consume_hold_release_service"
        ).value
        self.hold_release_poll_sec = float(
            self.get_parameter("hold_release_poll_sec").value
        )
        self.get_review_pending_service = self.get_parameter(
            "get_review_pending_service"
        ).value
        self.review_pending_poll_sec = float(
            self.get_parameter("review_pending_poll_sec").value
        )
        self.review_pending_wait_timeout_sec = float(
            self.get_parameter("review_pending_wait_timeout_sec").value
        )

        self.pose_loader.json_path = self.pass_place_goal_path
        self.cycle_index = 0
        self.last_pass_goal_mtime = self._get_pass_goal_mtime()
        self.last_pass_pose = None
        self.last_pass_label = "pass"
        self.last_successful_pass_pose = None
        self.last_successful_pass_label = "pass"
        self.pass_goal_applied_this_cycle = False
        self.phase_publisher = self.create_publisher(
            String,
            self.task_phase_topic,
            10,
        )
        self.consume_hold_release_client = self.create_client(
            Trigger,
            self.consume_hold_release_service,
        )
        self.get_review_pending_client = self.create_client(
            Trigger,
            self.get_review_pending_service,
        )

        self.get_logger().info(
            f"HRI adaptive task 준비 완료: {self.pass_place_goal_path}"
        )

    def publish_task_phase(self, phase):
        msg = String()
        msg.data = phase
        self.phase_publisher.publish(msg)
        self.get_logger().info(f"[PHASE] {phase}")

    def wait_for_hold_release(self):
        self.get_logger().info(
            "[HOLD] 외부 hold release trigger를 기다립니다."
        )

        if not self.consume_hold_release_client.wait_for_service(
            timeout_sec=5.0
        ):
            raise RuntimeError(
                "hold release consume 서비스를 사용할 수 없습니다"
            )

        while rclpy.ok():
            future = self.consume_hold_release_client.call_async(
                Trigger.Request()
            )
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if not future.done():
                time.sleep(max(self.hold_release_poll_sec, 0.01))
                continue

            response = future.result()
            if response is None:
                time.sleep(max(self.hold_release_poll_sec, 0.01))
                continue

            if response.success:
                self.get_logger().info("[HOLD] release trigger 수신")
                return

            time.sleep(max(self.hold_release_poll_sec, 0.01))

        raise RuntimeError("ROS 종료로 hold release 대기를 중단했습니다")

    def wait_for_review_completion(self):
        """RETURNING 종료 시점에 review pending이 있으면 완료까지 대기한다."""
        if not self.get_review_pending_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(
                "review pending 조회 서비스를 사용할 수 없습니다"
            )

        deadline = None
        if self.review_pending_wait_timeout_sec > 0.0:
            deadline = time.time() + self.review_pending_wait_timeout_sec

        while rclpy.ok():
            future = self.get_review_pending_client.call_async(
                Trigger.Request()
            )
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if not future.done():
                time.sleep(max(self.review_pending_poll_sec, 0.01))
                continue

            response = future.result()
            if response is None:
                time.sleep(max(self.review_pending_poll_sec, 0.01))
                continue

            if not response.success:
                self.get_logger().info(
                    "[REVIEW] pending 없음, 다음 cycle 준비를 계속합니다."
                )
                return

            self.get_logger().info(
                "[REVIEW] HRI 평가/JSON 생성 중이라 대기합니다."
            )
            if deadline is not None and time.time() >= deadline:
                self.get_logger().warn(
                    "review pending 대기 timeout 도달, 현재 목표 유지로 진행합니다."
                )
                return

            time.sleep(max(self.review_pending_poll_sec, 0.01))

        raise RuntimeError("ROS 종료로 review pending 대기를 중단했습니다")

    def _open_keyboard_input(self):
        try:
            return open("/dev/tty", "r", encoding="utf-8")
        except OSError:
            return None

    def wait_for_space_before_gripper_open(self, label):
        """그리퍼 open 직전에만 SPACE 입력을 기다린다."""
        if not self.manual_gripper_open_with_space:
            return

        self.get_logger().info(
            f"[GRIPPER OPEN WAIT] {label} - SPACE를 누르면 open, q로 중단"
        )
        input_file = self._open_keyboard_input()
        if input_file is None:
            self.get_logger().warn(
                "인터랙티브 터미널이 없어 SPACE 대기를 건너뜁니다."
            )
            return

        old_settings = termios.tcgetattr(input_file)
        try:
            tty.setcbreak(input_file.fileno())
            while rclpy.ok():
                readable, _, _ = select.select([input_file], [], [], 0.1)
                if not readable:
                    continue

                key = input_file.read(1)
                if key == " ":
                    return
                if key.lower() == "q":
                    raise KeyboardInterrupt
        finally:
            termios.tcsetattr(input_file, termios.TCSADRAIN, old_settings)
            input_file.close()

    def _get_pass_goal_mtime(self):
        """pass goal 파일의 수정 시간을 반환한다. 없으면 None."""
        try:
            return os.path.getmtime(self.pass_place_goal_path)
        except OSError:
            return None

    def _has_new_pass_goal(self, mtime):
        """노드 시작 또는 마지막 적용 이후 새 JSON인지 확인한다."""
        if mtime is None:
            return False
        if self.last_pass_goal_mtime is None:
            return True
        return mtime > self.last_pass_goal_mtime

    def _try_load_new_hri_pass_pose(self):
        """새 pass_place_goal.json이 있으면 pose로 변환한다."""
        deadline = None
        if self.pass_goal_wait_timeout_sec > 0.0:
            deadline = time.time() + self.pass_goal_wait_timeout_sec

        while rclpy.ok():
            now = time.time()
            mtime = self._get_pass_goal_mtime()
            if self._has_new_pass_goal(mtime):
                try:
                    pose = self.pose_loader.get_pass_place_pose()
                except (
                    OSError,
                    json.JSONDecodeError,
                    KeyError,
                    TypeError,
                    ValueError,
                ) as exc:
                    self.get_logger().warn(
                        f"HRI pass JSON 파싱 실패, 직전 pass 위치 유지: {exc}"
                    )
                    return None
                else:
                    self.last_pass_goal_mtime = mtime
                    return pose

            if deadline is None or now >= deadline:
                return None

            time.sleep(max(self.pass_goal_poll_sec, 0.01))

        raise RuntimeError("ROS 종료로 HRI pass JSON 확인을 중단했습니다")

    def get_hri_pass_target_pose(self):
        """cycle 시작 시 사용할 pass pose를 결정한다."""
        self.pass_goal_applied_this_cycle = False

        if self.use_yaml_pass_first_cycle and self.cycle_index == 1:
            self.get_logger().info("첫 cycle: YAML 기본 pass 위치 사용")
            self.last_pass_pose = self.pose_loader.get_pose("pass")
            self.last_pass_label = "pass"
            return self.last_pass_pose, self.last_pass_label

        pose = self._try_load_new_hri_pass_pose()
        if pose is not None:
            self.last_pass_pose = pose
            self.last_pass_label = "hri_pass_place"
            self.pass_goal_applied_this_cycle = True
            self.get_logger().info(
                "HRI pass JSON 적용: "
                f"frame_id={pose.header.frame_id}, "
                f"x={pose.pose.position.x:.3f}, "
                f"y={pose.pose.position.y:.3f}, "
                f"z={pose.pose.position.z:.3f}, "
                f"qx={pose.pose.orientation.x:.4f}, "
                f"qy={pose.pose.orientation.y:.4f}, "
                f"qz={pose.pose.orientation.z:.4f}, "
                f"qw={pose.pose.orientation.w:.4f}"
            )
            return self.last_pass_pose, self.last_pass_label

        if self.last_pass_pose is None:
            self.last_pass_pose = self.pose_loader.get_pose("pass")
            self.last_pass_label = "pass"

        self.get_logger().info(
            f"새 HRI pass JSON 없음: 직전 pass 위치 유지({self.last_pass_label})"
        )
        return self.last_pass_pose, self.last_pass_label

    def run_pick_and_place(self):
        """기본 시퀀스는 유지하되 pass 목표만 cycle 시작 시 JSON으로 결정한다."""
        self.cycle_index += 1
        self.publish_task_phase("PICKING")
        self.get_logger().info(
            f"=== HRI Adaptive Pick and Pass Cycle {self.cycle_index} 시작 ==="
        )

        if not self.gripper.wait_for_servers(timeout_sec=3.0):
            raise RuntimeError("Gripper 서비스가 모두 준비되지 않았습니다")
        if not self.moveit.wait_for_servers(timeout_sec=30.0):
            raise RuntimeError("MoveIt 액션/서비스 서버를 사용할 수 없습니다")
        if not self.moveit.wait_for_joint_state(timeout_sec=10.0):
            raise RuntimeError("joint_states를 사용할 수 없습니다")
        self.setup_planning_scene()

        pass_pose, pass_label = self.get_hri_pass_target_pose()
        if self.pass_goal_applied_this_cycle:
            self.get_logger().info(
                "이전 시점에 생성된 pass goal JSON을 이번 시퀀스에 반영했습니다."
            )

        self.wait_step("ready_pick + gripper open")
        self.move_to_joint_target("ready_pick")
        if not self.gripper.open():
            raise RuntimeError("그리퍼 열기 실패")

        self.wait_step("pre_pick -> pick")
        self.move_to_pose("pre_pick")
        self.move_to_pose("pick")

        self.wait_step("gripper close")
        if not self.gripper.close():
            raise RuntimeError("그리퍼 닫기 실패")

        self.wait_step("pre_pick -> ready_pick")
        self.move_to_pose("pre_pick")
        self.move_to_joint_target("ready_pick")

        self.wait_step("ready_pass")
        self.move_to_joint_target("ready_pass")

        self.wait_step(f"ready_pass -> {pass_label}")
        try:
            self.move_to_pose_stamped(pass_pose, pass_label)
        except RuntimeError as exc:
            fallback_pose = self.last_successful_pass_pose
            fallback_label = self.last_successful_pass_label
            if fallback_pose is None or fallback_pose is pass_pose:
                raise RuntimeError(
                    f"{pass_label} 실패. 이전 성공 pass pose가 없어 종료합니다: {exc}"
                ) from exc

            self.get_logger().warn(
                f"{pass_label} 실패. 이전 성공 pass pose({fallback_label})로 fallback pass를 시도합니다: {exc}"
            )
            self.move_to_pose_stamped(
                fallback_pose,
                f"{fallback_label}_fallback",
            )
            pass_pose = fallback_pose
            pass_label = fallback_label

        self.last_successful_pass_pose = pass_pose
        self.last_successful_pass_label = pass_label

        self.publish_task_phase("AT_TASK")
        self.wait_for_hold_release()

        self.wait_step("gripper open release")
        if not self.gripper.open():
            raise RuntimeError("물체 release를 위한 그리퍼 열기 실패")

        self.publish_task_phase("RETURNING")
        self.wait_step("ready_pass -> ready_pick")
        self.move_to_joint_target("ready_pass")
        self.move_to_joint_target("ready_pick")
        self.wait_for_review_completion()
        self.publish_task_phase("IDLE")

        self.get_logger().info(
            f"=== HRI Adaptive Pick and Pass Cycle {self.cycle_index} 완료 ==="
        )

    def run_repeat(self):
        """HRI pass JSON을 cycle마다 새로 읽으며 반복 실행한다."""
        if self.repeat_count < 1:
            self.get_logger().warn("repeat_count가 1보다 작아서 실행하지 않습니다")
            return

        self.get_logger().info(
            f"HRI adaptive 반복 실행 시작: {self.repeat_count}회"
        )

        for cycle_index in range(self.repeat_count):
            self.run_pick_and_place()

            is_last_cycle = cycle_index + 1 >= self.repeat_count
            if not is_last_cycle and self.cycle_wait_sec > 0.0:
                time.sleep(self.cycle_wait_sec)

        self.get_logger().info("HRI adaptive 반복 실행 완료")


def main(args=None):
    rclpy.init(args=args)

    node = HRIAdaptiveTaskNode()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        time.sleep(2.0)

        if node.auto_start:
            node.run_repeat()
        else:
            node.get_logger().info(
                "auto_start가 false입니다. 노드는 준비됐지만 작업은 시작하지 않습니다."
            )

    except KeyboardInterrupt:
        pass
    except Exception as exc:
        node.get_logger().error(f"HRI adaptive task 실패: {exc}")
    finally:
        node.get_logger().info("hri_adaptive_task 종료 중")
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        executor_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
