import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from turtlebot_interfaces.msg import Ui2Taskplanner
from turtlebot_interfaces.msg import Refiner2taskplanner


# ─────────────────────────────────────────────────────────────
# Util
# ─────────────────────────────────────────────────────────────

def quaternion_to_yaw(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


# ─────────────────────────────────────────────────────────────
# Config
# ─────────────────────────────────────────────────────────────

class Config:
    def __init__(self):
        self.hz = 10.0

        self.ui_topic = "/ui_msg"
        self.refiner_topic = "/refiner"
        self.odom_topic = "/odom"
        self.cmd_vel_topic = "/cmd_vel"

        self.navigate_to_pose = "/navigate_to_pose"


# ─────────────────────────────────────────────────────────────
# FSM Node
# ─────────────────────────────────────────────────────────────

class TurtlebotFSM(Node):

    def __init__(self):
        super().__init__("turtlebot_fsm")

        self.config = Config()

        # 전체 FSM 상태
        self._state = "IDLE"

        # deliver 내부 상태
        self.state_deliver = "SearchToGo"

        self._load_params()

        # UI 값
        self.deliver_object_id = None
        self.deliver_flag = False
        self.putback_flag = False

        # Refiner 값
        self.object_x = None
        self.object_y = None

        # Odom 값
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Nav2 goal 중복 전송 방지
        self.nav_running = False

        # Action client
        self.nav_client = ActionClient(self, NavigateToPose, self.config.navigate_to_pose)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, self.config.cmd_vel_topic, 10)

        # Subscriber
        self.ui_sub = self.create_subscription(Ui2Taskplanner, self.config.ui_topic, self._ui_callback, 10)
        self.refiner_sub = self.create_subscription(Refiner2taskplanner, self.config.refiner_topic, self._refiner_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, self.config.odom_topic, self._odom_callback, 10)

        # Timer
        timer_period = 1.0 / self.config.hz
        self.create_timer(timer_period, self._step)

        self.get_logger().info("TurtlebotFSM ready")
        self.get_logger().info(f"hz: {self.config.hz}")
        self.get_logger().info(f"timer_period: {timer_period:.3f} sec")
        self.get_logger().info(f"ui_topic: {self.config.ui_topic}")
        self.get_logger().info(f"refiner_topic: {self.config.refiner_topic}")
        self.get_logger().info(f"odom_topic: {self.config.odom_topic}")
        self.get_logger().info(f"cmd_vel_topic: {self.config.cmd_vel_topic}")
        self.get_logger().info(f"navigate_to_pose: {self.config.navigate_to_pose}")

    # ─────────────────────────────────────────────────────────
    # Parameter
    # ─────────────────────────────────────────────────────────

    def _load_params(self):
        self.declare_parameter("hz", 10.0)

        self.declare_parameter("topic.ui", "/ui_msg")
        self.declare_parameter("topic.refiner", "/refiner")
        self.declare_parameter("topic.odom", "/odom")
        self.declare_parameter("topic.cmd_vel", "/cmd_vel")

        self.declare_parameter("action.navigate_to_pose", "/navigate_to_pose")

        self.config.hz = float(
            self.get_parameter("hz").value
        )

        if self.config.hz <= 0.0:
            self.get_logger().warn(
                f"invalid hz: {self.config.hz}, use default 10.0"
            )
            self.config.hz = 10.0

        self.config.ui_topic = str(
            self.get_parameter("topic.ui").value
        )

        self.config.refiner_topic = str(
            self.get_parameter("topic.refiner").value
        )

        self.config.odom_topic = str(
            self.get_parameter("topic.odom").value
        )

        self.config.cmd_vel_topic = str(
            self.get_parameter("topic.cmd_vel").value
        )

        self.config.navigate_to_pose = str(
            self.get_parameter("action.navigate_to_pose").value
        )

    # ─────────────────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────────────────

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
            self._state = "DELIVER"
            self.state_deliver = "SearchToGo"
            self.nav_running = False

            self.get_logger().info("[FSM] IDLE → DELIVER")
            self.get_logger().info("[DELIVER] Start → SearchToGo")

        elif self.putback_flag:
            self._state = "PUT_BACK"
            self.get_logger().info("[FSM] IDLE → PUT_BACK")

    def _refiner_callback(self, msg: Refiner2taskplanner):
        if msg.object_x == 0.0 and msg.object_y == 0.0:
            self.object_x = None
            self.object_y = None
            return

        self.object_x = msg.object_x
        self.object_y = msg.object_y

        self.get_logger().info(
            f"[REFINER] object_x={self.object_x:.3f}, "
            f"object_y={self.object_y:.3f}"
        )

    def _odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    # ─────────────────────────────────────────────────────────
    # Nav2
    # ─────────────────────────────────────────────────────────

    def send_goal(self, x, y, qz, qw):
        if self.nav_running:
            return

        self.get_logger().info("Waiting for Nav2 action server...")

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
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

        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            self.nav_running = False
            return

        self.get_logger().info("Goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        result = future.result()
        status = result.status

        self.nav_running = False

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Navigation succeeded")
        else:
            self.get_logger().warn(
                f"Navigation failed or finished with status: {status}"
            )

        # 성공/실패 상관없이 SearchToGo 끝나면 Around로 이동
        if self.state_deliver == "SearchToGo":
            self.state_deliver = "Around"
            self.get_logger().info("[FSM] SearchToGo → Around")

    # ─────────────────────────────────────────────────────────
    # cmd_vel
    # ─────────────────────────────────────────────────────────

    def publish_cmd(self, linear_x, angular_z):
        msg = Twist()

        msg.linear.x = float(linear_x)
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(angular_z)

        self.cmd_pub.publish(msg)

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0)

    # ─────────────────────────────────────────────────────────
    # FSM Tick
    # ─────────────────────────────────────────────────────────

    def _step(self):
        match self._state:

            case "IDLE":
                return

            case "DELIVER":
                self._step_deliver()

            case "PUT_BACK":
                self._step_put_back()

            case _:
                self.get_logger().warn(f"Unknown FSM state: {self._state}")

    def _step_deliver(self):
        if self.deliver_object_id is None:
            return

        if not self.deliver_flag:
            return

        match self.state_deliver:

            case "SearchToGo":
                self.get_logger().info(
                    f"[SearchToGo] Searching object id={self.deliver_object_id}"
                )

                self.send_goal(
                    x=-0.8116069436073303,
                    y=-0.10050240904092789,
                    qz=0.13871737311756624,
                    qw=0.9903320101841412
                )

            case "Room1":
                self.get_logger().info("[Room1] Going to Room1")
                self.send_goal(
                    x=-0.8116069436073303,
                    y=-0.10050240904092789,
                    qz=0.13871737311756624,
                    qw=0.9903320101841412
                )

            case "Room2":
                self.get_logger().info("[Room2] Going to Room2")
                self.send_goal(
                    x=-0.8116069436073303,
                    y=-0.10050240904092789,
                    qz=0.13871737311756624,
                    qw=0.9903320101841412
                )
            
            case "Room3":
                self.get_logger().info("[Room3] Going to Room3")
                self.send_goal(
                    x=-0.8116069436073303,
                    y=-0.10050240904092789,
                    qz=0.13871737311756624,
                    qw=0.9903320101841412
                )

            case _:
                self.get_logger().warn(
                    f"Unknown deliver state: {self.state_deliver}"
                )

    def _step_put_back(self):
        self.get_logger().info("[PUT_BACK] not implemented yet")


# ─────────────────────────────────────────────────────────────
# main
# ─────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)

    node = TurtlebotFSM()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()