"""
Simple repeat runner for Indy7 pick-and-place.

현재 task_node.py의 run_pick_and_place()를 SPACE 입력 없이 N회 반복한다.
trajectory 기록/비교 없이 시뮬에서 여러 번 눈으로 확인하는 용도다.

실행 예시:
  # MoveIt, controller, gripper node를 먼저 실행한 뒤 launch로 반복 실행한다.
  ros2 launch indy7_task task_only.launch.py \
    task_executable:=task_repeat_node \
    repeat_count:=30 \
    cycle_wait_sec:=0.2 \
    max_velocity:=0.3 \
    max_acceleration:=0.3

  # ros2 run으로 직접 실행할 수도 있다.
  ros2 run indy7_task task_repeat_node --ros-args \
    -p repeat_count:=30 \
    -p cycle_wait_sec:=0.2 \
    -p max_velocity:=0.3 \
    -p max_acceleration:=0.3
"""

import time
from threading import Thread

import rclpy
from rclpy.executors import MultiThreadedExecutor

from indy7_task.task_node import Indy7TaskNode


class Indy7TaskRepeatNode(Indy7TaskNode):
    """run_pick_and_place()를 repeat_count만큼 반복 실행하는 노드."""

    def __init__(self):
        super().__init__()

        self.declare_parameter("repeat_count", 10)
        self.declare_parameter("cycle_wait_sec", 0.5)

        self.repeat_count = int(self.get_parameter("repeat_count").value)
        self.cycle_wait_sec = float(
            self.get_parameter("cycle_wait_sec").value
        )

    def run_repeat(self):
        if self.repeat_count < 1:
            self.get_logger().warn("repeat_count가 1보다 작아서 실행하지 않습니다")
            return

        self.get_logger().info(
            f"Pick-and-place 반복 실행 시작: {self.repeat_count}회"
        )

        for cycle_index in range(self.repeat_count):
            cycle_number = cycle_index + 1
            self.get_logger().info(
                f"========== Cycle {cycle_number}/{self.repeat_count} =========="
            )

            self.run_pick_and_place()

            if cycle_number < self.repeat_count and self.cycle_wait_sec > 0.0:
                time.sleep(self.cycle_wait_sec)

        self.get_logger().info("Pick-and-place 반복 실행 완료")


def main(args=None):
    rclpy.init(args=args)

    node = Indy7TaskRepeatNode()

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
                "auto_start가 false입니다. 노드는 준비됐지만 반복 작업은 시작하지 않습니다."
            )

    except KeyboardInterrupt:
        pass
    except Exception as exc:
        node.get_logger().error(f"반복 작업 실패: {exc}")
    finally:
        node.get_logger().info("indy7_task_repeat_node 종료 중")
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        executor_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
