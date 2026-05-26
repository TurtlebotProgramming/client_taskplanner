from dataclasses import dataclass, field
from typing import List, Tuple

@dataclass
class Blackboard:
    # 감지
    has_detection: bool = False
    detection: object = None
    target_class_id: int = -1  # ui_msg에서 받은 목표 클래스 ID

    # 로봇 포즈 (odom 기준, 미터/라디안)
    robot_x: float = 0.0
    robot_y: float = 0.0
    robot_yaw: float = 0.0

    # 경로 (미터 단위 waypoint 목록)
    path: List[Tuple[float, float]] = field(default_factory=list)

    # 속도 명령 출력
    cmd_linear_x: float = 0.0
    cmd_angular_z: float = 0.0

    # Pure Pursuit 파라미터
    lookahead_dist: float = 0.3
    goal_tolerance: float = 0.1
    max_linear_vel: float = 0.2
    max_angular_vel: float = 1.5
    dist_scale: float = 0.5  # 감속 시작 거리 (m)

    # 현재 목표 좌표
    current_goal: object = None

    # lookahead 시각화용
    lookahead_x: float = 0.0
    lookahead_y: float = 0.0

    goal_tolerance: float = 0.1  # 목표 도달 허용 오차 (m)
# 싱글톤 인스턴스
bb = Blackboard()
