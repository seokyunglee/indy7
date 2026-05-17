"""
Indy7 Planning Scene Client
===========================
MoveIt planning scene에 작업 환경과 물체를 등록하기 위한 얇은 helper다.

이 파일은 아직 task 정책을 담지 않는다. pick table, puzzle object,
handover zone 같은 "무엇을 언제 넣을지"는 나중에 scene_loader.py나
task_node.py에서 결정하고, 여기서는 MoveIt scene 조작 API만 제공한다.

기본 역할:
  1. CollisionObject 생성(box/cylinder)
  2. apply_planning_scene 서비스로 world object 추가/삭제
  3. 그리퍼에 object attach/detach
  4. 실험 초기화를 위한 scene clear
"""

import copy
import time

from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    PlanningScene,
)
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.callback_groups import ReentrantCallbackGroup
from shape_msgs.msg import SolidPrimitive


def _pose_from_xyz_quat(position, orientation=None):
    """(x, y, z)와 Quaternion으로 Pose를 만든다."""
    pose = Pose()
    pose.position = Point(
        x=float(position[0]),
        y=float(position[1]),
        z=float(position[2]),
    )
    if orientation is None:
        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    else:
        pose.orientation = copy.deepcopy(orientation)
    return pose


def make_box_collision_object(
    object_id,
    frame_id,
    position,
    dimensions,
    orientation=None,
    operation=CollisionObject.ADD,
):
    """박스 CollisionObject를 생성한다.

    dimensions는 MoveIt SolidPrimitive.BOX 규약대로 [x, y, z] 크기다.
    테이블, 선반, 고정 픽 장, 단순한 금지 영역을 만들 때 먼저 쓴다.
    """
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
    collision_object.primitives.append(box)
    collision_object.primitive_poses.append(
        _pose_from_xyz_quat(position, orientation)
    )
    return collision_object


def make_cylinder_collision_object(
    object_id,
    frame_id,
    position,
    height,
    radius,
    orientation=None,
    operation=CollisionObject.ADD,
):
    """원기둥 CollisionObject를 생성한다.

    dimensions는 MoveIt SolidPrimitive.CYLINDER 규약대로 [height, radius]다.
    컵, 기둥형 obstacle, 단순화한 사람/마커 영역 표현에 쓸 수 있다.
    """
    collision_object = CollisionObject()
    collision_object.header.frame_id = frame_id
    collision_object.id = object_id
    collision_object.operation = operation

    cylinder = SolidPrimitive()
    cylinder.type = SolidPrimitive.CYLINDER
    cylinder.dimensions = [float(height), float(radius)]
    collision_object.primitives.append(cylinder)
    collision_object.primitive_poses.append(
        _pose_from_xyz_quat(position, orientation)
    )
    return collision_object


class Indy7PlanningSceneClient:
    """ApplyPlanningScene 서비스를 감싼 Indy7 planning scene helper."""

    def __init__(self, node):
        self.node = node
        self.callback_group = ReentrantCallbackGroup()

        self.frame_id = self._declare_and_get("scene_frame_id", "link0")
        self.attached_link_name = self._declare_and_get(
            "attached_link_name",
            "tcp",
        )
        self.apply_service_name = self._declare_and_get(
            "apply_planning_scene_service",
            "apply_planning_scene",
        )

        self.apply_client = self.node.create_client(
            ApplyPlanningScene,
            self.apply_service_name,
            callback_group=self.callback_group,
        )

    # ----------------------------------------------------------
    #  Parameters / waiting
    # ----------------------------------------------------------
    def _declare_and_get(self, name, default_value):
        if not self.node.has_parameter(name):
            self.node.declare_parameter(name, default_value)
        return self.node.get_parameter(name).value

    def wait_for_service(self, timeout_sec=10.0):
        """apply_planning_scene 서비스가 준비될 때까지 기다린다."""
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

    # ----------------------------------------------------------
    #  Low-level scene diff
    # ----------------------------------------------------------
    def apply_scene_diff(
        self,
        world_objects=None,
        attached_objects=None,
        timeout_sec=5.0,
    ):
        """PlanningScene diff를 적용한다.

        world_objects는 테이블/장애물처럼 월드에 고정된 객체,
        attached_objects는 그리퍼에 붙은 물체를 표현한다.
        """
        if not self.apply_client.service_is_ready():
            if not self.wait_for_service(timeout_sec=timeout_sec):
                return False

        scene = PlanningScene()
        scene.is_diff = True

        if world_objects:
            scene.world.collision_objects = list(world_objects)
        if attached_objects:
            scene.robot_state.attached_collision_objects = list(
                attached_objects
            )
            scene.robot_state.is_diff = True

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

    # ----------------------------------------------------------
    #  World collision objects
    # ----------------------------------------------------------
    def add_collision_object(self, collision_object, timeout_sec=5.0):
        """이미 만들어진 CollisionObject를 world에 추가한다."""
        return self.apply_scene_diff(
            world_objects=[collision_object],
            timeout_sec=timeout_sec,
        )

    def add_box(
        self,
        object_id,
        position,
        dimensions,
        frame_id=None,
        orientation=None,
        timeout_sec=5.0,
    ):
        """박스 world object를 생성해 planning scene에 추가한다."""
        collision_object = make_box_collision_object(
            object_id=object_id,
            frame_id=frame_id or self.frame_id,
            position=position,
            dimensions=dimensions,
            orientation=orientation,
        )
        return self.add_collision_object(collision_object, timeout_sec)

    def add_cylinder(
        self,
        object_id,
        position,
        height,
        radius,
        frame_id=None,
        orientation=None,
        timeout_sec=5.0,
    ):
        """원기둥 world object를 생성해 planning scene에 추가한다."""
        collision_object = make_cylinder_collision_object(
            object_id=object_id,
            frame_id=frame_id or self.frame_id,
            position=position,
            height=height,
            radius=radius,
            orientation=orientation,
        )
        return self.add_collision_object(collision_object, timeout_sec)

    def remove_collision_object(
        self,
        object_id,
        frame_id=None,
        timeout_sec=5.0,
    ):
        """world에서 object 하나를 제거한다."""
        collision_object = CollisionObject()
        collision_object.header.frame_id = frame_id or self.frame_id
        collision_object.id = object_id
        collision_object.operation = CollisionObject.REMOVE
        return self.apply_scene_diff(
            world_objects=[collision_object],
            timeout_sec=timeout_sec,
        )

    def clear_scene(self, timeout_sec=5.0):
        """world collision object를 모두 제거한다.

        MoveIt 예제와 같은 관례로 빈 id + REMOVE를 보낸다. 환경 구성
        실험을 반복할 때 시작 상태를 정리하는 용도다.
        """
        collision_object = CollisionObject()
        collision_object.header.frame_id = self.frame_id
        collision_object.id = ""
        collision_object.operation = CollisionObject.REMOVE
        return self.apply_scene_diff(
            world_objects=[collision_object],
            timeout_sec=timeout_sec,
        )

    # ----------------------------------------------------------
    #  Attach / detach
    # ----------------------------------------------------------
    def attach_object(
        self,
        object_id,
        link_name=None,
        touch_links=None,
        frame_id=None,
        timeout_sec=5.0,
    ):
        """world object를 그리퍼 link에 부착한다.

        touch_links는 그리퍼 손가락처럼 접촉을 허용할 link 목록이다.
        실제 gripper link 이름이 정해지면 task 쪽에서 넘기는 구조로 둔다.
        """
        attached_object = AttachedCollisionObject()
        attached_object.link_name = link_name or self.attached_link_name
        attached_object.object.id = object_id
        attached_object.object.header.frame_id = frame_id or self.frame_id
        attached_object.object.operation = CollisionObject.ADD
        if touch_links:
            attached_object.touch_links = list(touch_links)

        return self.apply_scene_diff(
            attached_objects=[attached_object],
            timeout_sec=timeout_sec,
        )

    def detach_object(
        self,
        object_id,
        link_name=None,
        frame_id=None,
        timeout_sec=5.0,
    ):
        """그리퍼 link에 부착된 object를 분리한다."""
        attached_object = AttachedCollisionObject()
        attached_object.link_name = link_name or self.attached_link_name
        attached_object.object.id = object_id
        attached_object.object.header.frame_id = frame_id or self.frame_id
        attached_object.object.operation = CollisionObject.REMOVE

        return self.apply_scene_diff(
            attached_objects=[attached_object],
            timeout_sec=timeout_sec,
        )
