from ..blackboard.blackboard import Blackboard


class SearchState:
    def __init__(self):
        self._wp_idx = 0
        self._send_nav_goal = None
        self._bb = None
        self._logger = None
        self._goal_sent = False

    def init(self, bb: Blackboard, send_nav_goal, logger) -> None:
        self._bb = bb
        self._send_nav_goal = send_nav_goal
        self._logger = logger
        self._wp_idx = 0
        self._send_waypoint()

    def tick(self, bb: Blackboard, logger) -> None:
        if not self._goal_sent:
            self._send_waypoint()

    def _on_waypoint_reached(self, ok: bool):
        self._goal_sent = False
        self._wp_idx = (self._wp_idx + 1) % len(self._bb.search.search_waypoints)

    def _send_waypoint(self) -> None:
        wp = self._bb.search.search_waypoints[self._wp_idx]
        self._bb.current_goal = wp
        self._logger.info(f"[SEARCH] 웨이포인트 {self._wp_idx}: {wp}")
        self._goal_sent = True
        self._send_nav_goal(*wp, self._on_waypoint_reached)
