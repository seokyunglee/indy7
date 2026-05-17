"""
Indy7 Gripper Service Client
============================
indy7_gripper 노드가 제공하는 Trigger 서비스를 호출해 그리퍼를 제어한다.

사용 서비스:
  /gripper/open
  /gripper/close
  /gripper/state

실행 방법:
  이 파일은 직접 실행하지 않는다.
  task_node.py 또는 task_node_servo.py에서 GripperClient로 사용한다.
"""

import time

from std_srvs.srv import Trigger


class GripperClient:
    """open/close/state Trigger 서비스를 감싼 gripper helper."""

    def __init__(
        self,
        node,
        open_service="/gripper/open",
        close_service="/gripper/close",
        state_service="/gripper/state",
    ):
        self.node = node
        self.open_service = open_service
        self.close_service = close_service
        self.state_service = state_service

        self.open_client = node.create_client(Trigger, open_service)
        self.close_client = node.create_client(Trigger, close_service)
        self.state_client = node.create_client(Trigger, state_service)

    # ----------------------------------------------------------
    #  Service readiness
    # ----------------------------------------------------------
    def wait_for_servers(self, timeout_sec=5.0):
        """그리퍼 서비스들이 준비될 때까지 기다린다."""
        start_time = time.monotonic()
        clients = [
            (self.open_service, self.open_client),
            (self.close_service, self.close_client),
            (self.state_service, self.state_client),
        ]

        for service_name, client in clients:
            remaining = timeout_sec - (time.monotonic() - start_time)
            if remaining <= 0.0:
                self.node.get_logger().error(
                    f"Timed out waiting for {service_name}"
                )
                return False
            if not client.wait_for_service(timeout_sec=remaining):
                self.node.get_logger().error(
                    f"Service not available: {service_name}"
                )
                return False

        self.node.get_logger().info("Gripper services are available")
        return True

    # ----------------------------------------------------------
    #  Trigger service calls
    # ----------------------------------------------------------
    def _call(self, client, label, timeout_sec=5.0):
        """Trigger request를 보내고 success/message를 확인한다."""
        self.node.get_logger().info(f"Gripper {label} request")
        future = client.call_async(Trigger.Request())
        start_time = time.monotonic()

        while not future.done():
            if time.monotonic() - start_time > timeout_sec:
                self.node.get_logger().error(f"Gripper {label} timed out")
                return False
            time.sleep(0.02)

        response = future.result()
        if response is None:
            self.node.get_logger().error(
                f"Gripper {label} failed: no response"
            )
            return False

        if response.success:
            self.node.get_logger().info(
                f"Gripper {label} done: {response.message}"
            )
            return True

        self.node.get_logger().error(
            f"Gripper {label} failed: {response.message}"
        )
        return False

    def open(self):
        """그리퍼 열기."""
        return self._call(self.open_client, "open")

    def close(self):
        """그리퍼 닫기."""
        return self._call(self.close_client, "close")

    def state(self):
        """그리퍼 상태 요청."""
        return self._call(self.state_client, "state")
