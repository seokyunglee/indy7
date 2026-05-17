"""
Indy7 Task Pose Loader
======================
task_poses.yaml과 pass_place_goal.json을 읽어 MoveIt 목표 PoseStamped로 변환한다.

YAML pose 형식:
  poses:
    pick:
      frame_id: "link0"
      position: [x, y, z]
      orientation: [x, y, z, w]

JSON pass_place 형식:
  {
    "frame_id": "link0",
    "position": {"x": ..., "y": ..., "z": ...},
    "orientation": {"x": ..., "y": ..., "z": ..., "w": ...}
  }

실행 방법:
  이 파일은 직접 실행하지 않는다.
  task_node.py에서 PoseLoader로 사용한다.
"""

import json
import yaml

from geometry_msgs.msg import PoseStamped


def make_pose_stamped(node, frame_id, position, orientation):
    """position/orientation 리스트를 PoseStamped 메시지로 변환한다."""
    pose = PoseStamped()
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.header.frame_id = frame_id

    pose.pose.position.x = float(position[0])
    pose.pose.position.y = float(position[1])
    pose.pose.position.z = float(position[2])

    pose.pose.orientation.x = float(orientation[0])
    pose.pose.orientation.y = float(orientation[1])
    pose.pose.orientation.z = float(orientation[2])
    pose.pose.orientation.w = float(orientation[3])

    return pose


class PoseLoader:
    """YAML/JSON 좌표 파일을 읽는 작은 helper."""

    def __init__(self, node, yaml_path: str, json_path: str = ""):
        self.node = node
        self.yaml_path = yaml_path
        self.json_path = json_path
        self.yaml_data = self._load_yaml(yaml_path)

    def _load_yaml(self, path: str):
        """task_poses.yaml을 로드한다."""
        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f)

    def get_pose(self, name: str) -> PoseStamped:
        """YAML poses 섹션에서 이름 pose를 찾아 PoseStamped로 반환한다."""
        poses = self.yaml_data.get("poses", {})
        if name not in poses:
            raise KeyError(f"Pose '{name}' not found in {self.yaml_path}")

        data = poses[name]
        return make_pose_stamped(
            node=self.node,
            frame_id=data["frame_id"],
            position=data["position"],
            orientation=data["orientation"],
        )

    def get_task_param(self, key: str, default=None):
        """YAML task 섹션의 scalar parameter를 반환한다."""
        task = self.yaml_data.get("task", {})
        return task.get(key, default)

    def get_pass_place_pose(self) -> PoseStamped:
        """JSON pass_place_goal 파일을 PoseStamped로 반환한다."""
        if not self.json_path:
            raise ValueError("pass_place_goal json path is empty")

        with open(self.json_path, "r", encoding="utf-8") as f:
            data = json.load(f)

        pos = data["position"]
        ori = data["orientation"]

        return make_pose_stamped(
            node=self.node,
            frame_id=data["frame_id"],
            position=[pos["x"], pos["y"], pos["z"]],
            orientation=[ori["x"], ori["y"], ori["z"], ori["w"]],
        )
