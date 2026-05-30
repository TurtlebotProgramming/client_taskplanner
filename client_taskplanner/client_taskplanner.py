import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from turtlebot_interfaces.msg import Ui2Taskplanner

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class Config:
    def __init__(self):
        self.hz = 10.0
        self.ui_topic = "/ui"


class TurtlebotFSM(Node):

    def __init__(self):
        super().__init__("turtlebot_fsm")

        self.config = Config()
        self._state = "IDLE"

        self._load_params()

        self.deliver_object_id = None
        self.deliver_flag = False
        self.putback_flag = False

        self.state_deliver = "Search"

        # goal 중복 전송 방지용
        self.nav_running = False

        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.ui_sub = self.create_subscription(Ui2Taskplanner, self.config.ui_topic, self._ui_callback, 10)

        timer_period = 1.0 / self.config.hz
        self.create_timer(timer_period, self._step)

        self.get_logger().info("TurtlebotFSM ready")
        self.get_logger().info(f"hz: {self.config.hz}")
        self.get_logger().info(f"timer_period: {timer_period:.3f} sec")
        self.get_logger().info(f"ui_topic: {self.config.ui_topic}")

    # ── 파라미터 로드 ───────────────────────────────────────────────────────d
    def _load_params(self):
        self.declare_parameter("hz", 10.0)
        self.declare_parameter("topic.ui", "/ui")

        self.config.hz = float(self.get_parameter("hz").value)
        self.config.ui_topic = str(self.get_parameter("topic.ui").value)

    # ── UI 콜백 ─────────────────────────────────────────────────────────────
    def _ui_callback(self, msg: Ui2Taskplanner):
        self.deliver_object_id = msg.deliver_object_id
        self.deliver_flag = msg.deliver_flag
        self.putback_flag = msg.putback_flag

        self.get_logger().info(
            f"Received UI message: "
            f"deliver_object_id={self.deliver_object_id}, "
            f"deliver_flag={self.deliver_flag}, "
            f"putback_flag={self.putback_flag}"
        )

        if self.deliver_flag and not self.putback_flag:
            self.state_deliver = "Search"
            self.nav_running = False
            self.get_logger().info("[FSM] Deliver task started → Search")

    # ── Nav2 goal 전송 ──────────────────────────────────────────────────────
    def send_goal(self, x, y, qz, qw):
        if self.nav_running:
            return

        self.get_logger().info("Waiting for Nav2 action server...")

        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 action server not found")
            return

        goal_msg = NavigateToPose.Goal()

        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0

        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = float(qz)
        goal_msg.pose.pose.orientation.w = float(qw)

        self.nav_running = True

        self.get_logger().info(
            f"Sending goal: x={x}, y={y}, qz={qz}, qw={qw}"
        )

        future = self.client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            self.nav_running = False
            return

        self.get_logger().info("Goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        status = result.status

        self.nav_running = False

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Navigation succeeded")
        else:
            self.get_logger().warn(
                f"Navigation failed or finished with status: {status}"
            )

        # 성공/실패 상관없이 Search 단계가 끝나면 다음 상태로 이동
        if self.state_deliver == "Search":
            self.state_deliver = "GoToObject"
            self.get_logger().info("[FSM] Search → GoToObject")

    # ── tick ────────────────────────────────────────────────────────────────

    def _step(self):
        if self.deliver_object_id is None:
            return

        if not self.deliver_flag and not self.putback_flag:
            self.get_logger().info("No task received. Waiting for UI input...")
            return

        if self.deliver_flag and not self.putback_flag:
            match self.state_deliver:

                case "Search":
                    self.get_logger().info(
                        f"State: {self.state_deliver} - "
                        f"Searching for object {self.deliver_object_id}..."
                    )

                    self.send_goal(
                        x=-0.8116069436073303,
                        y=-0.10050240904092789,
                        qz=0.13871737311756624,
                        qw=0.9903320101841412
                    )

                case "GoToObject":
                    self.get_logger().info(
                        f"State: {self.state_deliver} - "
                        f"Going to object {self.deliver_object_id}..."
                    )


def main(args=None):
    rclpy.init(args=args)

    node = TurtlebotFSM()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()