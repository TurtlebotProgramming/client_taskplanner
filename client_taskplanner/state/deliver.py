from ..blackboard.blackboard import Blackboard


class DeliverState:
    def __init__(self):
        self._bb = None
        self._send_gripper = None
        self._logger = None

    def init(self, bb: Blackboard, send_nav_goal, logger) -> None:
        self._bb = bb
        bb.deliver.delivered = False
        self._logger = logger
        pos = bb.config.deliver_position
        bb.current_goal = pos
        logger.info(f"[DELIVER] 목표: {pos}")
        send_nav_goal(*pos, self._on_nav_done)

    def tick(self, send_gripper) -> None:
        self._send_gripper = send_gripper

    def _on_nav_done(self, ok: bool):
        if not ok or self._send_gripper is None:
            return
        self._bb.deliver.delivered = True
        self._logger.info("[DELIVER] 목적지 도달, 그리퍼 release 전송")
        self._send_gripper(0)
