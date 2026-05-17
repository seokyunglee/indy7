"""
Indy7 Step-by-Step Pick & Place Task Node
=========================================
task_node.py의 8단계 pick-and-place 사이클을 그대로 사용하되,
각 단계마다 터미널 스페이스바 입력을 기다린다.

실행 방법:
  # MoveIt(move_group), controller, gripper node가 먼저 실행되어 있어야 한다.
  ros2 launch indy7_task task_only.launch.py task_executable:=task_node_servo

조작:
  SPACE : 다음 단계 실행
  q     : task 중단
"""

import select
import termios
import tty
from threading import Thread

import rclpy
from rclpy.executors import MultiThreadedExecutor

from indy7_task.task_node import Indy7TaskNode


class Indy7TaskServoNode(Indy7TaskNode):
    """스페이스바로 한 단계씩 진행하는 task node."""

    STEPS_PER_CYCLE = 8

    def __init__(self):
        super().__init__()
        self.cycle_index = 0
        self.step_index = 0

    # ----------------------------------------------------------
    #  Keyboard input
    # ----------------------------------------------------------
    def _open_keyboard_input(self):
        try:
            return open("/dev/tty", "r", encoding="utf-8")
        except OSError:
            return None

    # ----------------------------------------------------------
    #  Cycle and step logging
    # ----------------------------------------------------------
    def run_pick_and_place(self):
        self.cycle_index += 1
        self.step_index = 0
        self.get_logger().info(
            f"========== Cycle {self.cycle_index} start =========="
        )

        try:
            super().run_pick_and_place()
        finally:
            self.get_logger().info(
                f"========== Cycle {self.cycle_index} end =========="
            )

    def wait_step(self, label: str):
        """각 단계마다 SPACE 입력을 기다린다."""
        self.step_index += 1
        self.get_logger().info(
            f"[CYCLE {self.cycle_index} | "
            f"STEP {self.step_index}/{self.STEPS_PER_CYCLE}] {label}"
        )
        self.get_logger().info("Press SPACE to continue, or q to stop.")

        input_file = self._open_keyboard_input()
        if input_file is None:
            self.get_logger().warn(
                "No interactive terminal detected. "
                "Pressing SPACE is unavailable, so the task will continue."
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


def main(args=None):
    """ROS entry point."""
    rclpy.init(args=args)

    node = Indy7TaskServoNode()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        if node.auto_start:
            node.run_pick_and_place()
        else:
            node.get_logger().info(
                "auto_start is false. Node is ready, but task will not start."
            )
    except KeyboardInterrupt:
        node.get_logger().info("Task servo interrupted")
    except Exception as exc:
        node.get_logger().error(f"Task servo failed: {exc}")
    finally:
        node.get_logger().info("Shutting down indy7_task_servo_node")
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        executor_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
