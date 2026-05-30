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
from turtlebot_interfaces.srv import Gripper
from client_vision_interfaces.msg import TurtlebotDetection


def quaternion_to_yaw(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class Config:
    def __init__(self):
        self.hz = 10.0

        self.ui_topic = "/ui_msg"
        self.refiner_topic = "/vision2taskplanner"
        self.detection_topic = "/detection"
        self.odom_topic = "/odom"
        self.cmd_vel_topic = "/cmd_vel"

        self.navigate_to_pose = "/navigate_to_pose"
        self.gripper_service = "/gripper/service"


class TurtlebotFSM(Node):

    def __init__(self):
        super().__init__("turtlebot_fsm")

        self.config = Config()

        self._state = "IDLE"
        self.state_deliver = "OpenGripper"

        self._load_params()

        # UI 값
        self.deliver_object_id = None
        self.deliver_flag = False
        self.putback_flag = False

        # Refiner object 위치
        self.object_x = None
        self.object_y = None

        # Detection 값
        self.bbox_center_x = None
        self.bbox_width = None
        self.bbox_height = None
        self.bbox_area = None

        # class_id별 정지 기준 bbox width(px)
        # 실제 로봇 테스트하면서 값 조정 필요
        self.stop_bbox_width_by_class = {
            0: 700.0,  # can
            1: 700.0,  # bottle
            2: 700.0,  # box
        }
        self.default_stop_bbox_width = 700.0

        # Odom 현재 위치
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # 중복 실행 방지 플래그
        self.nav_running = False
        self.gripper_running = False
        self.open_done = False
        self.grip_done = False

        # 방 목록
        self.rooms = [
            {
                "name": "Room1",
                "x": 0.7011054158210754,
                "y": 0.13435018062591553,
                "qz": -0.009558199717154485,
                "qw": 0.9999543193657233,
            },
            {
                "name": "Room2",
                "x": 0.9548425674438477,
                "y": 0.861286461353302,
                "qz": 0.3655982603888551,
                "qw": 0.9307727499237625,
            },
            {
                "name": "Room3",
                "x": 0.092291921377182,
                "y": 0.571383535861969,
                "qz": 0.7025614486148799,
                "qw": 0.7116230820597105,
            },
        ]

        # Room1 → Room2 사이에만 거칠 중간 경유지
        self.room1_to_room2_waypoint = {
            "name": "Room1ToRoom2Waypoint",
            "x": 0.21259844303131104,
            "y": 0.17552828788757324,
            "qz": 0.3746956168154849,
            "qw": 0.9271478818070304,
        }

        self.room_index = 0

        # Action client
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            self.config.navigate_to_pose
        )

        # Service client
        self.gripper_client = self.create_client(
            Gripper,
            self.config.gripper_service
        )

        # Publisher
        self.cmd_pub = self.create_publisher(
            Twist,
            self.config.cmd_vel_topic,
            10
        )

        # Subscriber
        self.ui_sub = self.create_subscription(
            Ui2Taskplanner,
            self.config.ui_topic,
            self._ui_callback,
            10
        )

        self.refiner_sub = self.create_subscription(
            Refiner2taskplanner,
            self.config.refiner_topic,
            self._refiner_callback,
            1
        )

        self.detection_sub = self.create_subscription(
            TurtlebotDetection,
            self.config.detection_topic,
            self._detection_callback,
            1
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            self.config.odom_topic,
            self._odom_callback,
            10
        )

        timer_period = 1.0 / self.config.hz
        self.create_timer(timer_period, self._step)

        self.get_logger().info("TurtlebotFSM ready")
        self.get_logger().info(f"hz: {self.config.hz}")
        self.get_logger().info(f"timer_period: {timer_period:.3f} sec")
        self.get_logger().info(f"ui_topic: {self.config.ui_topic}")
        self.get_logger().info(f"refiner_topic: {self.config.refiner_topic}")
        self.get_logger().info(f"detection_topic: {self.config.detection_topic}")
        self.get_logger().info(f"odom_topic: {self.config.odom_topic}")
        self.get_logger().info(f"cmd_vel_topic: {self.config.cmd_vel_topic}")
        self.get_logger().info(f"navigate_to_pose: {self.config.navigate_to_pose}")
        self.get_logger().info(f"gripper_service: {self.config.gripper_service}")

    def _load_params(self):
        self.declare_parameter("hz", 10.0)

        self.declare_parameter("topic.ui", "/ui_msg")
        self.declare_parameter("topic.refiner", "/vision2taskplanner")
        self.declare_parameter("topic.detection", "/detection")
        self.declare_parameter("topic.odom", "/odom")
        self.declare_parameter("topic.cmd_vel", "/cmd_vel")

        self.declare_parameter("action.navigate_to_pose", "/navigate_to_pose")
        self.declare_parameter("service.gripper", "/gripper/service")

        self.config.hz = float(self.get_parameter("hz").value)

        if self.config.hz <= 0.0:
            self.get_logger().warn(
                f"invalid hz: {self.config.hz}, use default 10.0"
            )
            self.config.hz = 10.0

        self.config.ui_topic = str(self.get_parameter("topic.ui").value)
        self.config.refiner_topic = str(self.get_parameter("topic.refiner").value)
        self.config.detection_topic = str(self.get_parameter("topic.detection").value)
        self.config.odom_topic = str(self.get_parameter("topic.odom").value)
        self.config.cmd_vel_topic = str(self.get_parameter("topic.cmd_vel").value)

        self.config.navigate_to_pose = str(
            self.get_parameter("action.navigate_to_pose").value
        )

        self.config.gripper_service = str(
            self.get_parameter("service.gripper").value
        )

    def _clear_detection_data(self):
        self.bbox_center_x = None
        self.bbox_width = None
        self.bbox_height = None
        self.bbox_area = None

    def _clear_object_data(self):
        self.object_x = None
        self.object_y = None
        self._clear_detection_data()

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
            self.state_deliver = "OpenGripper"

            self.room_index = 0
            self._clear_object_data()

            self.nav_running = False
            self.gripper_running = False
            self.open_done = False
            self.grip_done = False

            self.get_logger().info("[FSM] IDLE → DELIVER")
            self.get_logger().info("[DELIVER] Start → OpenGripper")

        elif self.putback_flag:
            self._state = "PUT_BACK"
            self.get_logger().info("[FSM] IDLE → PUT_BACK")

    def _refiner_callback(self, msg: Refiner2taskplanner):
        if msg.object_x == 0.0 and msg.object_y == 0.0:
            if self.state_deliver not in ["GoToObject", "Grip", "Done"]:
                self.object_x = None
                self.object_y = None
            return

        self.object_x = msg.object_x
        self.object_y = msg.object_y

        self.get_logger().info(
            f"[REFINER] object_x={self.object_x:.3f}, "
            f"object_y={self.object_y:.3f}"
        )

    def _detection_callback(self, msg: TurtlebotDetection):
        if self.deliver_object_id is None or not msg.class_ids:
            self._clear_detection_data()
            return

        candidates = [
            i for i, cid in enumerate(msg.class_ids)
            if cid == self.deliver_object_id
        ]

        if not candidates:
            self._clear_detection_data()
            return

        best_idx = max(candidates, key=lambda i: msg.score[i])

        x1 = float(msg.x1[best_idx])
        x2 = float(msg.x2[best_idx])
        y1 = float(msg.y1[best_idx])
        y2 = float(msg.y2[best_idx])

        self.bbox_center_x = (x1 + x2) / 2.0
        self.bbox_width = x2 - x1
        self.bbox_height = y2 - y1
        self.bbox_area = self.bbox_width * self.bbox_height

        self.get_logger().info(
            f"[DETECTION] class={self.deliver_object_id}, "
            f"center_x={self.bbox_center_x:.1f}, "
            f"w={self.bbox_width:.1f}, "
            f"h={self.bbox_height:.1f}, "
            f"area={self.bbox_area:.1f}"
        )

    def _odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = quaternion_to_yaw(msg.pose.pose.orientation)

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
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Goal response error: {e}")
            self.nav_running = False
            self._after_nav_done()
            return

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            self.nav_running = False
            self._after_nav_done()
            return

        self.get_logger().info("Goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        try:
            result = future.result()
            status = result.status
        except Exception as e:
            self.get_logger().error(f"Navigation result error: {e}")
            status = None

        self.nav_running = False

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Navigation succeeded")
        else:
            self.get_logger().warn(
                f"Navigation failed or finished with status: {status}"
            )

        self._after_nav_done()

    def _after_nav_done(self):
        if self._state != "DELIVER":
            return

        if self.state_deliver == "RoomMove":
            if self.room_index >= len(self.rooms):
                self.get_logger().warn("[DELIVER] invalid room_index")
                self._state = "IDLE"
                return

            room_name = self.rooms[self.room_index]["name"]
            self.state_deliver = "CheckObject"
            self.get_logger().info(f"[FSM] {room_name} 이동 완료 → CheckObject")
            return

        if self.state_deliver == "Room1ToRoom2Waypoint":
            self.state_deliver = "RoomMove"
            self.get_logger().info("[FSM] Room1ToRoom2Waypoint 도착 → Room2 이동")
            return

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

    def send_gripper(self, object_name: str, command: int):
        if self.gripper_running:
            return

        if not self.gripper_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                f"[GRIPPER] service not ready: {self.config.gripper_service}"
            )
            return

        req = Gripper.Request()
        req.object_name = str(object_name)
        req.command = int(command)

        self.gripper_running = True

        self.get_logger().info(
            f"[GRIPPER] request: service={self.config.gripper_service}, "
            f"object_name={req.object_name}, command={req.command}"
        )

        future = self.gripper_client.call_async(req)
        future.add_done_callback(self._gripper_response_callback)

    def _gripper_response_callback(self, future):
        self.gripper_running = False

        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"[GRIPPER] service error: {e}")
            return

        if result.success:
            self.get_logger().info(f"[GRIPPER] success: {result.message}")

            if self.state_deliver == "OpenGripper":
                self.open_done = True
                self.state_deliver = "RoomMove"
                self.get_logger().info("[FSM] OpenGripper → RoomMove")
                return

            if self.state_deliver == "Grip":
                self.grip_done = True
                self.state_deliver = "Done"
                self.get_logger().info("[FSM] Grip → Done")
                return

        else:
            self.get_logger().warn(f"[GRIPPER] failed: {result.message}")

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

            case "OpenGripper":
                self.stop_robot()

                if self.open_done:
                    self.state_deliver = "RoomMove"
                    return

                # release command
                # 서버가 release에서 object_name을 무시하지 않는다면 can 같은 기본값 사용
                self.send_gripper(
                    object_name="can",
                    command=0
                )

            case "RoomMove":
                if self.room_index >= len(self.rooms):
                    self.stop_robot()
                    self.get_logger().warn("[DELIVER] 모든 방 탐색 실패")
                    self._state = "IDLE"
                    return

                room = self.rooms[self.room_index]

                self.get_logger().info(
                    f"[RoomMove] Going to {room['name']}"
                )

                self.send_goal(
                    x=room["x"],
                    y=room["y"],
                    qz=room["qz"],
                    qw=room["qw"],
                )

            case "Room1ToRoom2Waypoint":
                waypoint = self.room1_to_room2_waypoint

                self.get_logger().info(
                    f"[Waypoint] Going to {waypoint['name']}"
                )

                self.send_goal(
                    x=waypoint["x"],
                    y=waypoint["y"],
                    qz=waypoint["qz"],
                    qw=waypoint["qw"],
                )

            case "CheckObject":
                room = self.rooms[self.room_index]
                room_name = room["name"]

                if self.object_x is not None and self.object_y is not None:
                    self.get_logger().info(
                        f"[CheckObject] {room_name}에서 물체 발견 → GoToObject"
                    )
                    self.state_deliver = "GoToObject"
                    return

                self.get_logger().warn(
                    f"[CheckObject] {room_name}에 물체 없음 → 다음 방"
                )

                prev_room_index = self.room_index

                self.room_index += 1
                self._clear_object_data()

                if self.room_index >= len(self.rooms):
                    self.stop_robot()
                    self.get_logger().warn("[DELIVER] 모든 방에서 물체 못 찾음")
                    self._state = "IDLE"
                    return

                next_room = self.rooms[self.room_index]["name"]

                if prev_room_index == 0 and self.room_index == 1:
                    self.state_deliver = "Room1ToRoom2Waypoint"
                    self.get_logger().info(
                        "[FSM] Room1 → Room2 전 중간 경유지 이동"
                    )
                    return

                self.state_deliver = "RoomMove"
                self.get_logger().info(f"[FSM] 다음 방 이동 → {next_room}")

            case "GoToObject":
                IMAGE_CENTER_X = 320.0
                ALIGN_THRESH_PX = 20.0
                LINEAR_V = 0.1
                ANGULAR_V = 0.3

                if self.bbox_center_x is None:
                    self.stop_robot()
                    return

                pixel_error = self.bbox_center_x - IMAGE_CENTER_X

                if abs(pixel_error) > ALIGN_THRESH_PX:
                    sign = -1.0 if pixel_error > 0 else 1.0
                    self.publish_cmd(0.0, sign * ANGULAR_V)
                    return

                target_bbox_width = self.stop_bbox_width_by_class.get(
                    self.deliver_object_id,
                    self.default_stop_bbox_width
                )

                if self.bbox_width is not None and self.bbox_width >= target_bbox_width:
                    self.stop_robot()
                    self.state_deliver = "Grip"
                    self.get_logger().info(
                        f"[GoToObject] bbox_width={self.bbox_width:.1f} >= "
                        f"target={target_bbox_width:.1f} → Grip"
                    )
                    return

                self.get_logger().info(
                    f"[GoToObject] aligned, moving forward... "
                    f"bbox_width={self.bbox_width}, "
                    f"target={target_bbox_width}"
                )

                self.publish_cmd(LINEAR_V, 0.0)

            case "Grip":
                self.stop_robot()

                if self.grip_done:
                    return

                object_name_map = {
                    0: "can",
                    1: "bottle",
                    2: "box",
                }

                object_name = object_name_map.get(self.deliver_object_id)

                if object_name is None:
                    self.get_logger().warn(
                        f"[Grip] unknown object id: {self.deliver_object_id}"
                    )
                    self.state_deliver = "Done"
                    return

                self.send_gripper(
                    object_name=object_name,
                    command=1
                )

            case "Done":
                self.stop_robot()
                self.get_logger().info("[Done] delivery object grip completed")

            case _:
                self.get_logger().warn(
                    f"Unknown deliver state: {self.state_deliver}"
                )

    def _step_put_back(self):
        self.stop_robot()
        self.get_logger().info("[PUT_BACK] not implemented yet")


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