"""
Minimal MoveIt planning scene helper for static shelf collision.

The task node only needs to clear old objects and add one fixed box. Gazebo or
the real setup owns object interaction; MoveIt only gets this static obstacle.
"""

import time

from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.callback_groups import ReentrantCallbackGroup
from shape_msgs.msg import SolidPrimitive


def _make_box(object_id, frame_id, center, dimensions, operation):
    collision_object = CollisionObject()
    collision_object.header.frame_id = frame_id
    collision_object.id = object_id
    collision_object.operation = operation

    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [
        float(dimensions[0]),
        float(dimensions[1]),
        float(dimensions[2]),
    ]

    pose = Pose()
    pose.position = Point(
        x=float(center[0]),
        y=float(center[1]),
        z=float(center[2]),
    )
    pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

    collision_object.primitives.append(box)
    collision_object.primitive_poses.append(pose)
    return collision_object


class Indy7PlanningSceneClient:
    """ApplyPlanningScene client with only static box support."""

    def __init__(self, node):
        self.node = node
        self.callback_group = ReentrantCallbackGroup()
        self.frame_id = self._declare_and_get("scene_frame_id", "link0")
        self.apply_service_name = self._declare_and_get(
            "apply_planning_scene_service",
            "apply_planning_scene",
        )
        self.apply_client = self.node.create_client(
            ApplyPlanningScene,
            self.apply_service_name,
            callback_group=self.callback_group,
        )

    def _declare_and_get(self, name, default_value):
        if not self.node.has_parameter(name):
            self.node.declare_parameter(name, default_value)
        return self.node.get_parameter(name).value

    def wait_for_service(self, timeout_sec=10.0):
        if self.apply_client.wait_for_service(timeout_sec=timeout_sec):
            return True
        self.node.get_logger().error(
            f"{self.apply_service_name} service is unavailable"
        )
        return False

    def _wait_future(self, future, timeout_sec, label):
        start_time = time.monotonic()
        while not future.done():
            if time.monotonic() - start_time > timeout_sec:
                self.node.get_logger().error(f"Timed out waiting for {label}")
                return None
            time.sleep(0.02)
        return future.result()

    def apply_world_objects(self, world_objects, timeout_sec=5.0):
        if not self.apply_client.service_is_ready():
            if not self.wait_for_service(timeout_sec=timeout_sec):
                return False

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = list(world_objects)

        request = ApplyPlanningScene.Request()
        request.scene = scene

        response = self._wait_future(
            self.apply_client.call_async(request),
            timeout_sec,
            "apply_planning_scene response",
        )
        if response is None:
            return False
        if response.success:
            return True

        self.node.get_logger().error("Planning scene update failed")
        return False

    def add_box_by_top_center(
        self,
        object_id,
        top_center,
        dimensions,
        frame_id=None,
        timeout_sec=5.0,
    ):
        center = (
            float(top_center[0]),
            float(top_center[1]),
            float(top_center[2]) - float(dimensions[2]) * 0.5,
        )
        box = _make_box(
            object_id=object_id,
            frame_id=frame_id or self.frame_id,
            center=center,
            dimensions=dimensions,
            operation=CollisionObject.ADD,
        )
        return self.apply_world_objects([box], timeout_sec=timeout_sec)

    def clear_scene(self, timeout_sec=5.0):
        clear_object = CollisionObject()
        clear_object.header.frame_id = self.frame_id
        clear_object.id = ""
        clear_object.operation = CollisionObject.REMOVE
        return self.apply_world_objects([clear_object], timeout_sec=timeout_sec)
