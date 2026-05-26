import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose

from turtlebot_interfaces.msg import TurtlebotDetection, Ui2taskplanner
from turtlebot_interfaces.srv import Gripper
from ..blackboard.blackboard import bb
from ..inputs.inputs import odom_callback, refiner_callback
from ..outputs.outputs import send_gripper, send_nav_goal
from ..state.search import SearchState
from ..state.found import FoundState
from ..state.deliver import DeliverState
from ..state.put_back import PutBackState


class TurtlebotFSM(Node):

    def __init__(self):
        super().__init__('turtlebot_fsm')

        self._state = "IDLE"
        self._search = SearchState()
        self._found = FoundState()
        self._deliver = DeliverState()
        self._put_back = PutBackState()

        self.create_subscription(Ui2taskplanner, '/ui_msg', self._state_cb, 5)
        self.create_subscription(Odometry, '/odom', odom_callback, 1)
        self.create_subscription(TurtlebotDetection, '/detection', refiner_callback, 1)

        self._nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._gripper_client = self.create_client(Gripper, '/gripper')

        self.create_timer(0.1, self._step)
        self.get_logger().info("TurtlebotFSM ready")

    def _nav_goal(self, x: float, y: float, done_cb=None):
        send_nav_goal(self._nav_client, self.get_logger(), x, y, done_cb)

    def _gripper(self, command: int, done_cb=None):
        send_gripper(self._gripper_client, self.get_logger(), command, done_cb)

    def _on_found_grip_done(self, ok: bool):
        if ok:
            self._goto("DELIEVER")

    # ── 콜백 ─────────────────────────────────────────────────────────────────

    def _state_cb(self, msg: Ui2taskplanner):
        if msg.deliver_flag:
            bb.target_class_id = msg.deliver_object_id
            new = "SEARCH"
        elif msg.putback_flag:
            new = "PUT_BACK"
        else:
            return

        if new == self._state:
            return
        if new == "PUT_BACK" and not self._deliver.delivered:
            self.get_logger().warn("[FSM] PUT_BACK 거부: DELIVER 미완료")
            return
        self.get_logger().info(f"[FSM] {self._state} → {new}")
        self._on_enter(new)
        self._state = new

    # ── 상태 진입 ─────────────────────────────────────────────────────────────

    def _on_enter(self, state: str):
        match state:
            case "IDLE":
                pass
            case "SEARCH":
                self._search.init(bb, self._nav_goal, self.get_logger())
            case "FOUND":
                self._found.init(bb, self._nav_goal, self.get_logger())
            case "DELIEVER":
                self._deliver.init(bb, self._nav_goal, self.get_logger())
            case "PUT_BACK":
                self._put_back.init(bb, self._found.saved_object, self._nav_goal, self.get_logger(), self._gripper)

    # ── 전이 조건 ─────────────────────────────────────────────────────────────

    def _transition(self) -> None:
        match self._state:
            case "SEARCH":
                if bb.has_detection and bb.detection and \
                        bb.target_class_id in bb.detection.class_ids:
                    self._goto("FOUND")

    def _goto(self, state: str) -> None:
        self.get_logger().info(f"[FSM] {self._state} → {state}")
        self._on_enter(state)
        self._state = state

    # ── tick ─────────────────────────────────────────────────────────────────

    def _step(self):
        self._transition()
        match self._state:
            case "IDLE":
                pass
            case "SEARCH":
                self._search.tick(bb, self.get_logger())
            case "FOUND":
                self._found.tick(bb, self.get_logger(), self._gripper, self._on_found_grip_done)
            case "DELIEVER":
                self._deliver.tick(self._gripper)
            case "PUT_BACK":
                pass


def main():
    rclpy.init()
    rclpy.spin(TurtlebotFSM())
    rclpy.shutdown()
