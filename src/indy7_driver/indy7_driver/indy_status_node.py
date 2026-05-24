#!/usr/bin/env python3
"""Terminal-friendly Indy status monitor.

Run:
  ros2 run indy7_driver indy_status_node.py --ros-args \
    -p indy_ip:=166.104.xxx.xxx \
    -p poll_period_sec:=1.0

Services:
  ros2 service call /indy/status std_srvs/srv/Trigger "{}"
"""

import json
from threading import Lock

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


OP_STATE_NAMES = {
    0: "SYSTEM_OFF",
    1: "SYSTEM_ON",
    2: "VIOLATE",
    3: "RECOVER_HARD",
    4: "RECOVER_SOFT",
    5: "IDLE",
    6: "MOVING",
    7: "TEACHING",
    8: "COLLISION",
    9: "STOP_AND_OFF",
    10: "COMPLIANCE",
    11: "BRAKE_CONTROL",
    12: "SYSTEM_RESET",
    13: "SYSTEM_SWITCH",
    15: "VIOLATE_HARD",
    16: "MANUAL_RECOVER",
    17: "TELE_OP",
}


class IndyStatusNode(Node):
    """Read IndyDCP3 status and print it periodically for debugging."""

    def __init__(self):
        super().__init__("indy_status_node")

        self.declare_parameter("indy_ip", "166.104.234.72")
        self.declare_parameter("poll_period_sec", 1.0)
        self.declare_parameter("ready_op_state", 5)

        self.indy_ip = self.get_parameter("indy_ip").value
        self.poll_period_sec = float(
            self.get_parameter("poll_period_sec").value
        )
        self.ready_op_state = int(
            self.get_parameter("ready_op_state").value
        )
        self.lock = Lock()

        try:
            from neuromeka import IndyDCP3
        except Exception as exc:
            raise RuntimeError(
                "Failed to import neuromeka. Install it with: "
                "pip3 install --upgrade neuromeka"
            ) from exc

        self.get_logger().info(f"Connecting to Indy controller: {self.indy_ip}")
        self.indy = IndyDCP3(self.indy_ip)

        self.status_service = self.create_service(
            Trigger,
            "/indy/status",
            self.status_callback,
        )

        self.timer = None
        if self.poll_period_sec > 0.0:
            self.timer = self.create_timer(
                self.poll_period_sec,
                self.timer_callback,
            )

        self.get_logger().info(
            "Indy status monitor ready | "
            f"poll_period_sec={self.poll_period_sec}"
        )

    def _as_dict(self, data):
        if isinstance(data, dict):
            return data
        if data is None:
            return {}
        return {"raw": str(data)}

    def _as_int_or_none(self, value):
        try:
            return int(value)
        except (TypeError, ValueError):
            return None

    def _read_status(self):
        with self.lock:
            robot_data = self._as_dict(self.indy.get_robot_data())
            violation_data = self._as_dict(self.indy.get_violation_data())

        op_state = self._as_int_or_none(robot_data.get("op_state"))
        op_state_name = OP_STATE_NAMES.get(op_state, "UNKNOWN")
        violation_code = violation_data.get("violation_code")
        violation_str = violation_data.get("violation_str", "")

        return {
            "ready": op_state == self.ready_op_state,
            "op_state": op_state,
            "op_state_name": op_state_name,
            "violation_code": violation_code,
            "violation_str": violation_str,
            "robot_data": robot_data,
            "violation_data": violation_data,
        }

    def _status_message(self, status):
        return (
            f"op_state={status['op_state']}({status['op_state_name']}) "
            f"ready={status['ready']} "
            f"violation_code={status['violation_code']} "
            f"violation_str={status['violation_str']}"
        )

    def timer_callback(self):
        try:
            status = self._read_status()
            message = self._status_message(status)
            if status["ready"]:
                self.get_logger().info(message)
            else:
                self.get_logger().warn(message)
        except Exception as exc:
            self.get_logger().error(f"Failed to read Indy status: {exc}")

    def status_callback(self, request, response):
        try:
            status = self._read_status()
            response.success = True
            response.message = json.dumps(
                status,
                ensure_ascii=False,
                sort_keys=True,
            )
        except Exception as exc:
            response.success = False
            response.message = f"Failed to read Indy status: {exc}"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = IndyStatusNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
