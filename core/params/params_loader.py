import os
import yaml
from ament_index_python.packages import get_package_share_directory


def load_follower_params(node, bb) -> None:
    _load_common(node, bb)


def _load_common(node, bb) -> None:
    node.declare_parameter("ui2taskplanner_topic", "/ui_msg")
    node.declare_parameter("odom_topic", "/odom")
    node.declare_parameter("detection_topic", "/detection")
    node.declare_parameter("vision2taskplanner_topic", "/vision2taskplanner")
    node.declare_parameter("gripper_service", "/gripper")
    node.declare_parameter("navigate_to_pose", "/navigate_to_pose")

    node.bb.config.ui2taskplanner_topic = str(node.get_parameter("ui2taskplanner_topic").value)
    node.bb.config.odom_topic = str(node.get_parameter("odom_topic").value)
    node.bb.config.detection_topic = str(node.get_parameter("detection_topic").value)
    node.bb.config.vision2taskplanner_topic = str(node.get_parameter("vision2taskplanner_topic").value)
    node.bb.config.gripper_service = str(node.get_parameter("gripper_service").value)
    node.bb.config.navigate_to_pose = str(node.get_parameter("navigate_to_pose").value)


def load_target_params(node, bb) -> None:
    node.declare_parameter("goal_tolerance", 5.0)   # cm
    node.declare_parameter("deliver_position", [0.0, 0.0])

    goal_tolerance_cm = float(node.get_parameter("goal_tolerance").value)
    node.bb.config.goal_tolerance = goal_tolerance_cm / 100.0  # cm → m

    pos = node.get_parameter("deliver_position").value
    node.bb.config.deliver_position = (float(pos[0]), float(pos[1]))