import math
from ..blackboard.blackboard import Blackboard
from ..util.util import object_to_world_pos


class FoundState:
    def __init__(self):
        self._bb = None
        self._send_nav_goal = None
        self._send_gripper = None
        self._on_grip_done = None
        self._logger = None
        self._nav_done = False

    def init(self, bb: Blackboard, send_nav_goal, logger) -> None:
        self._bb = bb
        self._send_nav_goal = send_nav_goal
        self._logger = logger
        self._nav_done = False
        bb.found.object_world_pos = None
        goal = self._object_world_pos(bb)
        if goal:
            self._send_goal(bb, goal)
        else:
            logger.warn("[FOUND] 감지 정보 없음")

    def tick(self, bb: Blackboard, logger, send_gripper, on_grip_done=None) -> None:
        self._send_gripper = send_gripper
        self._on_grip_done = on_grip_done
        if self._nav_done:
            return
        goal = self._object_world_pos(bb)
        if goal and self._goal_moved(goal, bb.config.goal_tolerance):
            self._send_goal(bb, goal)

    def _send_goal(self, bb: Blackboard, goal: tuple) -> None:
        bb.current_goal = goal
        bb.found.object_world_pos = goal
        self._logger.info(f"[FOUND] 목표 갱신: ({goal[0]:.2f}, {goal[1]:.2f})")
        self._send_nav_goal(*goal, self._on_nav_done)

    def _goal_moved(self, new_goal: tuple, tolerance: float) -> bool:
        saved = self._bb.found.object_world_pos
        if saved is None:
            return True
        dx = new_goal[0] - saved[0]
        dy = new_goal[1] - saved[1]
        return math.sqrt(dx * dx + dy * dy) > tolerance

    def _object_world_pos(self, bb: Blackboard):
        if not bb.has_detection:
            return None
        return object_to_world_pos(bb.robot_x, bb.robot_y, bb.robot_yaw,
                                   bb.object_dist, bb.object_theta)

    def _on_nav_done(self, ok: bool):
        if not ok or self._send_gripper is None:
            return
        self._nav_done = True
        self._logger.info("[FOUND] 도착, 그리퍼 grip 전송")
        self._send_gripper(1, self._on_grip_done)
