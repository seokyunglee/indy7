"""Interface node that stores robot state and serves the HRI HTTP bridge."""

import json
import os
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

import rclpy
from rclpy.node import Node


HOST = "0.0.0.0"
PORT = 8000


class Indy7TaskInterfaceNode(Node):
    """Tracks a coarse robot phase and stores hold-release triggers."""

    def __init__(self):
        super().__init__("indy7_task_interface_node")

        self.declare_parameter("phase_topic", "/indy7/task_phase")
        self.declare_parameter("state_topic", "/indy7/robot_state")
        self.declare_parameter(
            "request_hold_release_service",
            "/indy7/request_hold_release",
        )
        self.declare_parameter(
            "consume_hold_release_service",
            "/indy7/consume_hold_release",
        )
        self.declare_parameter(
            "set_review_pending_service",
            "/indy7/set_review_pending",
        )
        self.declare_parameter(
            "get_review_pending_service",
            "/indy7/get_review_pending",
        )
        self.declare_parameter(
            "pass_goal_output_path",
            "/home/leeseo/Cobot_2x2x2_HRI_Experiment/json_log/pass_place_goal.json",
        )

        self.phase_topic = self.get_parameter("phase_topic").value
        self.state_topic = self.get_parameter("state_topic").value
        self.request_hold_release_service = self.get_parameter(
            "request_hold_release_service"
        ).value
        self.consume_hold_release_service = self.get_parameter(
            "consume_hold_release_service"
        ).value
        self.set_review_pending_service = self.get_parameter(
            "set_review_pending_service"
        ).value
        self.get_review_pending_service = self.get_parameter(
            "get_review_pending_service"
        ).value
        self.pass_goal_output_path = Path(
            self.get_parameter("pass_goal_output_path").value
        )
        self.pass_goal_tmp_path = self.pass_goal_output_path.with_name(
            f"{self.pass_goal_output_path.stem}.tmp{self.pass_goal_output_path.suffix}"
        )

        self.robot_state = "IDLE"
        self.hold_release_requested = False
        self.review_pending = False
        self.http_server = None
        self.http_thread = None

        self.state_publisher = self.create_publisher(String, self.state_topic, 10)
        self.phase_subscription = self.create_subscription(
            String,
            self.phase_topic,
            self._handle_phase,
            10,
        )
        self.request_service = self.create_service(
            Trigger,
            self.request_hold_release_service,
            self._handle_request_hold_release,
        )
        self.consume_service = self.create_service(
            Trigger,
            self.consume_hold_release_service,
            self._handle_consume_hold_release,
        )
        self.set_review_pending_srv = self.create_service(
            SetBool,
            self.set_review_pending_service,
            self._handle_set_review_pending,
        )
        self.get_review_pending_srv = self.create_service(
            Trigger,
            self.get_review_pending_service,
            self._handle_get_review_pending,
        )

        self._publish_state()
        self._start_http_server()
        self.get_logger().info(
            "interface node ready: "
            f"phase_topic={self.phase_topic}, "
            f"state_topic={self.state_topic}, "
            f"http=http://{HOST}:{PORT}"
        )

    def _publish_state(self):
        msg = String()
        msg.data = self.robot_state
        self.state_publisher.publish(msg)

    def _save_pass_goal_json(self, data):
        self.pass_goal_output_path.parent.mkdir(parents=True, exist_ok=True)
        text = json.dumps(data, ensure_ascii=False, indent=2)
        self.pass_goal_tmp_path.write_text(text, encoding="utf-8")
        os.replace(self.pass_goal_tmp_path, self.pass_goal_output_path)

    def _make_http_handler(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def _send_json(self, status_code, payload):
                body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
                self.send_response(status_code)
                self.send_header(
                    "Content-Type",
                    "application/json; charset=utf-8",
                )
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _read_json_body(self):
                length = int(self.headers.get("Content-Length", "0"))
                raw = self.rfile.read(length) if length > 0 else b"{}"
                return json.loads(raw.decode("utf-8"))

            def log_message(self, format, *args):
                return

            def do_POST(self):
                try:
                    if self.path == "/pass_goal":
                        data = self._read_json_body()
                        node._save_pass_goal_json(data)
                        node.get_logger().info(
                            f"pass goal updated: {node.pass_goal_output_path}"
                        )
                        self._send_json(
                            200,
                            {
                                "ok": True,
                                "saved_to": str(node.pass_goal_output_path),
                            },
                        )
                        return

                    if self.path == "/hold_finished":
                        node.hold_release_requested = True
                        node.get_logger().info(
                            "hold finished notified via HTTP"
                        )
                        self._send_json(
                            200,
                            {"ok": True, "message": "hold finished accepted"},
                        )
                        return

                    if self.path == "/review_pending":
                        data = self._read_json_body()
                        pending = bool(data.get("pending", False))
                        node.review_pending = pending
                        node.get_logger().info(
                            f"review_pending updated via HTTP -> {pending}"
                        )
                        self._send_json(
                            200,
                            {"ok": True, "pending": pending},
                        )
                        return

                    self._send_json(404, {"ok": False, "error": "not found"})
                except Exception as exc:
                    node.get_logger().error(f"HTTP POST error: {exc}")
                    self._send_json(400, {"ok": False, "error": str(exc)})

            def do_GET(self):
                try:
                    if self.path == "/current":
                        if not node.pass_goal_output_path.exists():
                            self._send_json(
                                404,
                                {"ok": False, "error": "no json"},
                            )
                            return
                        self.send_response(200)
                        self.send_header(
                            "Content-Type",
                            "application/json; charset=utf-8",
                        )
                        self.end_headers()
                        self.wfile.write(node.pass_goal_output_path.read_bytes())
                        return

                    if self.path == "/robot_state":
                        self._send_json(
                            200,
                            {"ok": True, "robot_state": node.robot_state},
                        )
                        return

                    self._send_json(200, {"ok": True, "message": "receiver alive"})
                except Exception as exc:
                    node.get_logger().error(f"HTTP GET error: {exc}")
                    self._send_json(400, {"ok": False, "error": str(exc)})

        return Handler

    def _start_http_server(self):
        handler = self._make_http_handler()
        self.http_server = HTTPServer((HOST, PORT), handler)
        self.http_thread = threading.Thread(
            target=self.http_server.serve_forever,
            daemon=True,
        )
        self.http_thread.start()

    def shutdown_http_server(self):
        if self.http_server is not None:
            self.http_server.shutdown()
            self.http_server.server_close()
        if self.http_thread is not None:
            self.http_thread.join(timeout=1.0)

    def _handle_phase(self, msg: String):
        new_state = msg.data.strip() or "IDLE"
        if new_state == "AT_TASK":
            self.hold_release_requested = False
        if new_state == "PICKING":
            self.review_pending = False

        if new_state != self.robot_state:
            self.robot_state = new_state
            self._publish_state()
            self.get_logger().info(f"robot_state -> {self.robot_state}")

    def _handle_request_hold_release(self, request, response):
        del request
        self.hold_release_requested = True
        response.success = True
        response.message = "hold release requested"
        self.get_logger().info("hold release requested by external client")
        return response

    def _handle_consume_hold_release(self, request, response):
        del request
        response.success = self.hold_release_requested
        if self.hold_release_requested:
            response.message = "hold release consumed"
            self.hold_release_requested = False
            self.get_logger().info("hold release consumed by task node")
        else:
            response.message = "hold release not requested"
        return response

    def _handle_set_review_pending(self, request, response):
        self.review_pending = bool(request.data)
        response.success = True
        response.message = (
            "review pending enabled"
            if self.review_pending
            else "review pending cleared"
        )
        self.get_logger().info(
            f"review_pending -> {self.review_pending}"
        )
        return response

    def _handle_get_review_pending(self, request, response):
        del request
        response.success = self.review_pending
        response.message = (
            "review pending"
            if self.review_pending
            else "review not pending"
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = Indy7TaskInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("interface node shutting down")
        node.shutdown_http_server()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
