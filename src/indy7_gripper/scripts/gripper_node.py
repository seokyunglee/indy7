#!/usr/bin/env python3
"""
Indy7 gripper service node.

이 파일이 실제 그리퍼 제어 서비스를 제공하는 서버다.
task 쪽의 GripperClient는 아래 서비스를 호출만 하고, 실제 sim/real 제어는
여기서 처리한다.

터미널 테스트:
  ros2 service call /gripper/open std_srvs/srv/Trigger "{}"
  ros2 service call /gripper/close std_srvs/srv/Trigger "{}"
  ros2 service call /gripper/half_open std_srvs/srv/Trigger "{}"
  ros2 service call /gripper/state std_srvs/srv/Trigger "{}"

또는 테스트 클라이언트:
  ros2 run indy7_gripper gripper_client_test.py open
  ros2 run indy7_gripper gripper_client_test.py close
"""

from __future__ import annotations

import time
from threading import Lock
from typing import Optional

import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class SimGripperBackend:
    """Gazebo/RViz 시뮬레이션용 그리퍼 backend.

    /gripper/* 서비스가 들어오면 gripper controller topic으로
    JointTrajectory를 발행해 손가락 joint 목표 위치를 보낸다.
    """

    def __init__(self, node: Node):
        self.node = node

        self.controller_topic = node.get_parameter("controller_topic").value
        self.left_joint = node.get_parameter("left_joint").value
        self.right_joint = node.get_parameter("right_joint").value

        self.open_width = float(node.get_parameter("open_width").value)
        self.half_width = float(node.get_parameter("half_width").value)
        self.close_width = float(node.get_parameter("close_width").value)

        self.motion_time_sec = float(node.get_parameter("motion_time_sec").value)

        self.left_sign = float(node.get_parameter("left_sign").value)
        self.right_sign = float(node.get_parameter("right_sign").value)

        self.current_width = self.close_width

        self.publisher = node.create_publisher(
            JointTrajectory,
            self.controller_topic,
            10,
        )

        self.node.get_logger().info(
            "SimGripperBackend ready | "
            f"topic={self.controller_topic}, "
            f"joints=[{self.left_joint}, {self.right_joint}], "
            f"open_width={self.open_width}"
        )

    def _publish_width(self, width: float) -> str:
        """목표 gripper width를 joint trajectory 명령으로 변환해 발행한다."""
        width = max(self.close_width, min(float(width), self.open_width))

        msg = JointTrajectory()
        msg.joint_names = [self.left_joint, self.right_joint]

        point = JointTrajectoryPoint()
        point.positions = [
            self.left_sign * width,
            self.right_sign * width,
        ]

        sec = int(self.motion_time_sec)
        nanosec = int((self.motion_time_sec - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)

        msg.points.append(point)

        self.publisher.publish(msg)
        self.current_width = width

        return (
            f"sim command sent | width={width:.4f} m | "
            f"positions={point.positions} | topic={self.controller_topic}"
        )

    def open(self) -> str:
        return self._publish_width(self.open_width)

    def close(self) -> str:
        return self._publish_width(self.close_width)

    def half_open(self) -> str:
        return self._publish_width(self.half_width)

    def state(self) -> str:
        return f"sim state | current_width={self.current_width:.4f} m"


class RealGripperBackend:
    """실물 MPLM1630 그리퍼용 backend.

    Neuromeka IndyDCP3 API로 endtool DO를 제어한다.
    open/close 상태 이름은 launch/config 파라미터에서 바꿀 수 있다.
    """

    def __init__(self, node: Node):
        self.node = node

        self.robot_ip = node.get_parameter("robot_ip").value
        self.port_name = node.get_parameter("port_name").value
        self.open_state_name = node.get_parameter("open_state").value
        self.close_state_name = node.get_parameter("close_state").value
        self.settle_sec = float(node.get_parameter("settle_sec").value)

        self.lock = Lock()

        try:
            from neuromeka import IndyDCP3
            from neuromeka.enums import EndtoolState
        except Exception as exc:
            raise RuntimeError(
                "Failed to import neuromeka package. "
                "Install it with: pip3 install --upgrade neuromeka"
            ) from exc

        self.EndtoolState = EndtoolState

        try:
            self.open_state = getattr(EndtoolState, self.open_state_name)
            self.close_state = getattr(EndtoolState, self.close_state_name)
        except AttributeError as exc:
            raise RuntimeError(
                f"Invalid EndtoolState. "
                f"open_state={self.open_state_name}, close_state={self.close_state_name}"
            ) from exc

        self.node.get_logger().info(f"Connecting to Indy controller at {self.robot_ip}")
        self.indy = IndyDCP3(self.robot_ip)

        self.node.get_logger().info(
            "RealGripperBackend ready | "
            f"port={self.port_name}, "
            f"open_state={self.open_state_name}, "
            f"close_state={self.close_state_name}"
        )

    def _write_state(self, state_value) -> str:
        """Indy controller의 endtool DO에 open/close 신호를 쓴다."""
        with self.lock:
            before = self.indy.get_endtool_do()
            result = self.indy.set_endtool_do([(self.port_name, [state_value])])
            time.sleep(self.settle_sec)
            after = self.indy.get_endtool_do()

        return f"before={before} | result={result} | after={after}"

    def open(self) -> str:
        return "real open sent | " + self._write_state(self.open_state)

    def close(self) -> str:
        return "real close sent | " + self._write_state(self.close_state)

    def half_open(self) -> str:
        return (
            "real half_open unsupported | "
            "MPLM1630 real backend supports open/close signal only"
        )

    def state(self) -> str:
        with self.lock:
            do_state = self.indy.get_endtool_do()
            di_state = self.indy.get_endtool_di()
        return f"real state | DO={do_state} | DI={di_state}"


class Indy7GripperNode(Node):
    """그리퍼 서비스를 등록하고 sim/real backend로 명령을 넘기는 ROS2 노드."""

    def __init__(self):
        super().__init__("indy7_gripper_node")

        self._declare_parameters()

        self.mode = self.get_parameter("mode").value

        self.backend: Optional[object] = None

        if self.mode == "sim":
            self.backend = SimGripperBackend(self)
        elif self.mode == "real":
            self.backend = RealGripperBackend(self)
        else:
            raise RuntimeError(f"Invalid mode: {self.mode}. Use 'sim' or 'real'.")

        self.open_srv = self.create_service(
            Trigger,
            "/gripper/open",
            self.open_callback,
        )
        self.close_srv = self.create_service(
            Trigger,
            "/gripper/close",
            self.close_callback,
        )
        self.half_open_srv = self.create_service(
            Trigger,
            "/gripper/half_open",
            self.half_open_callback,
        )
        self.state_srv = self.create_service(
            Trigger,
            "/gripper/state",
            self.state_callback,
        )

        self.get_logger().info(
            f"indy7_gripper_node ready | mode={self.mode} | "
            "services=[/gripper/open, /gripper/close, /gripper/half_open, /gripper/state]"
        )

    def _declare_parameters(self):
        # 공통 파라미터: sim이면 JointTrajectory, real이면 IndyDCP3 backend를 쓴다.
        self.declare_parameter("mode", "sim")

        # 시뮬레이션 파라미터: controller topic과 손가락 joint 이름/목표 폭.
        self.declare_parameter("controller_topic", "/gripper_controller/joint_trajectory")
        self.declare_parameter("left_joint", "left_finger_joint")
        self.declare_parameter("right_joint", "right_finger_joint")
        self.declare_parameter("open_width", 0.015)
        self.declare_parameter("half_width", 0.0075)
        self.declare_parameter("close_width", 0.0)
        self.declare_parameter("motion_time_sec", 1.0)
        self.declare_parameter("left_sign", 1.0)
        self.declare_parameter("right_sign", 1.0)

        # 실물 파라미터: Indy controller 접속 정보와 endtool DO 상태.
        self.declare_parameter("robot_ip", "166.104.234.72")
        self.declare_parameter("port_name", "C")
        self.declare_parameter("open_state", "HIGH_PNP")
        self.declare_parameter("close_state", "LOW_PNP")
        self.declare_parameter("settle_sec", 0.5)
        self.declare_parameter("model", "MPLM1630")

    def _handle_backend_call(self, response, fn_name: str):
        """Trigger callback 공통 처리.

        서비스 이름(open/close/state)에 맞는 backend method를 호출하고
        Trigger 응답의 success/message로 결과를 돌려준다.
        """
        try:
            fn = getattr(self.backend, fn_name)
            message = fn()
            response.success = True
            response.message = message
            self.get_logger().info(message)
        except Exception as exc:
            response.success = False
            response.message = f"{fn_name} failed: {exc}"
            self.get_logger().error(response.message)

        return response

    def open_callback(self, request, response):
        """터미널 명령: ros2 service call /gripper/open std_srvs/srv/Trigger "{}"."""
        return self._handle_backend_call(response, "open")

    def close_callback(self, request, response):
        """터미널 명령: ros2 service call /gripper/close std_srvs/srv/Trigger "{}"."""
        return self._handle_backend_call(response, "close")

    def half_open_callback(self, request, response):
        """터미널 명령: ros2 service call /gripper/half_open std_srvs/srv/Trigger "{}"."""
        return self._handle_backend_call(response, "half_open")

    def state_callback(self, request, response):
        """터미널 명령: ros2 service call /gripper/state std_srvs/srv/Trigger "{}"."""
        return self._handle_backend_call(response, "state")


def main(args=None):
    rclpy.init(args=args)

    node = Indy7GripperNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
