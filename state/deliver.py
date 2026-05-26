from ..blackboard.blackboard import Blackboard

DELIVER_POSITION = (0.0, 0.0)


class DeliverState:
    def __init__(self):
        self.delivered = False
        self._send_gripper = None
        self._logger = None

    def init(self, bb: Blackboard, send_nav_goal, logger) -> None:
        self.delivered = False
        self._logger = logger
        bb.current_goal = DELIVER_POSITION
        logger.info(f"[DELIVER] 목표: {DELIVER_POSITION}")
        send_nav_goal(*DELIVER_POSITION, self._on_nav_done)

    def tick(self, send_gripper) -> None:
        self._send_gripper = send_gripper

    def _on_nav_done(self, ok: bool):
        if not ok or self._send_gripper is None:
            return
        self.delivered = True
        self._logger.info("[DELIVER] 목적지 도달, 그리퍼 release 전송")
        self._send_gripper(0)
