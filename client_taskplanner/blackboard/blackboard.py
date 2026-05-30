from dataclasses import dataclass, field
from typing import List, Tuple


_SEARCH_WAYPOINTS = [(-0.1555812507867813, -0.4232494533061981, 0)] # x,y,z


@dataclass
class Config:
    ui2taskplanner_topic: str = '/ui_msg'
    odom_topic: str = '/odom'
    detection_topic: str = '/detection'
    vision2taskplanner_topic: str = '/vision2taskplanner'
    gripper_service: str = '/gripper'
    navigate_to_pose: str = '/navigate_to_pose'
    goal_tolerance: float = 0.05
    deliver_position: tuple = (0.6, 0.75)


@dataclass
class search:
    search_waypoints: List[Tuple[float, float]] = field(default_factory=_SEARCH_WAYPOINTS.copy)


@dataclass
class deliver:
    delivered: bool = False
    deliver_position: int = (0, 0)


@dataclass
class found:
    object_world_pos: int = None


@dataclass
class put_back:
    bottle_put_back_position: int = (0, 0)
    can_put_back_position: int = (0, 0)
    box_put_back_position: int = (0, 0)


@dataclass
class Blackboard:
    config: Config = field(default_factory=Config)

    # 감지
    has_detection: bool = False
    detection: object = None
    target_class_id: int = -1
    object_dist: float = 0.0
    object_theta: float = 0.0

    # 로봇 포즈 (odom 기준, 미터/라디안)
    robot_x: float = 0.0
    robot_y: float = 0.0
    robot_yaw: float = 0.0

    # 현재 목표 좌표
    current_goal: object = None

    # 상태별 데이터
    search: object = field(default_factory=search)
    deliver: object = field(default_factory=deliver)
    found: object = field(default_factory=found)
    put_back: object = field(default_factory=put_back)


# 싱글톤 인스턴스
bb = Blackboard()
