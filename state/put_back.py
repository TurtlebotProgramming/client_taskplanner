from ..blackboard.blackboard import Blackboard


class PutBackState:
    def __init__(self):
        self._send_gripper = None
        self._logger = None

    def init(self, bb: Blackboard, saved_object, send_nav_goal, logger, send_gripper) -> None:
        self._send_gripper = send_gripper
        self._logger = logger
        if saved_object is None:
            logger.warn("[PUT_BACK] 저장된 물체 좌표 없음")
            return
        bb.current_goal = saved_object
        logger.info(f"[PUT_BACK] 목표: ({saved_object[0]:.2f}, {saved_object[1]:.2f})")
        logger.info("[PUT_BACK] 그리퍼 grip 전송")
        send_gripper(1)
        send_nav_goal(*saved_object, self._on_nav_done)

    def tick(self, bb: Blackboard, logger, send_gripper) -> None:
        pass

    def _on_nav_done(self, ok: bool):
        if not ok or self._send_gripper is None:
            return
        self._logger.info("[PUT_BACK] 목적지 도달, 그리퍼 release 전송")
        self._send_gripper(0)
