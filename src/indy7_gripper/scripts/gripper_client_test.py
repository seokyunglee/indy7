#!/usr/bin/env python3

import sys

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class GripperClient(Node):
    def __init__(self, service_name: str):
        super().__init__("gripper_client_test")
        self.service_name = service_name
        self.cli = self.create_client(Trigger, service_name)

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Waiting for service {service_name} ...")

        self.req = Trigger.Request()

    def send_request(self):
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main(args=None):
    if len(sys.argv) < 2:
        print("Usage: ros2 run indy7_gripper gripper_client_test.py [open|close|half|state]")
        return

    cmd = sys.argv[1].strip().lower()

    service_map = {
        "open": "/gripper/open",
        "close": "/gripper/close",
        "half": "/gripper/half_open",
        "half_open": "/gripper/half_open",
        "state": "/gripper/state",
    }

    if cmd not in service_map:
        print("Usage: ros2 run indy7_gripper gripper_client_test.py [open|close|half|state]")
        return

    rclpy.init(args=args)

    node = GripperClient(service_map[cmd])
    result = node.send_request()

    print(result)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()