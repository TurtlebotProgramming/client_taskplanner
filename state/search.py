from ..blackboard.blackboard import Blackboard

WAYPOINTS = [
    (1.0,  0.0),
    (1.0,  1.0),
    (0.0,  1.0),
    (-1.0, 1.0),
    (-1.0, 0.0),
    (-1.0,-1.0),
    (0.0, -1.0),
    (1.0, -1.0),
]


class SearchState:
    def __init__(self):
        self._wp_idx = 0
        self._send_nav_goal = None
        self._bb = None
        self._logger = None

    def init(self, bb: Blackboard, send_nav_goal, logger) -> None:
        self._bb = bb
        self._send_nav_goal = send_nav_goal
        self._logger = logger
        self._wp_idx = 0
        self._send_waypoint()

    def tick(self, bb: Blackboard, logger) -> None:
        pass  # 웨이포인트 전진은 _on_wp_reached 콜백에서 처리

    def _on_wp_reached(self, ok: bool):
        self._wp_idx = (self._wp_idx + 1) % len(WAYPOINTS)
        self._send_waypoint()

    def _send_waypoint(self) -> None:
        wp = WAYPOINTS[self._wp_idx]
        self._bb.current_goal = wp
        self._logger.info(f"[SEARCH] 웨이포인트 {self._wp_idx}: {wp}")
        self._send_nav_goal(*wp, self._on_wp_reached)
