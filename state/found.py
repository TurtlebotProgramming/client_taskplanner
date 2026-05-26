from ..blackboard.blackboard import Blackboard


class FoundState:
    def __init__(self):
        self.saved_object = None
        self._send_gripper = None
        self._on_grip_done = None
        self._logger = None

    def init(self, bb: Blackboard, send_nav_goal, logger) -> None:
        self._logger = logger
        goal = self._object_world_pos(bb)
        if goal:
            bb.current_goal = goal
            self.saved_object = goal
            logger.info(f"[FOUND] 목표: ({goal[0]:.2f}, {goal[1]:.2f})")
            send_nav_goal(*goal, self._on_nav_done)
        else:
            logger.warn("[FOUND] 감지 정보 없음")

    def tick(self, bb: Blackboard, logger, send_gripper, on_grip_done=None) -> None:
        self._send_gripper = send_gripper
        self._on_grip_done = on_grip_done

    def _on_nav_done(self, ok: bool):
        if not ok or self._send_gripper is None:
            return
        self._logger.info("[FOUND] 도착, 그리퍼 grip 전송")
        self._send_gripper(1, self._on_grip_done)


