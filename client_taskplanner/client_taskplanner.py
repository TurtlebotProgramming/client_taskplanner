import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav2_msgs.action import BackUp
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
        self.backup_action = "/backup"
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

        # class_id별 bbox 기준 width(px)
        self.stop_bbox_width_by_class = {
            0: 600.0,  # can
            1: 600.0,  # bottle
            2: 600.0,  # box
        }
        self.default_stop_bbox_width = 700.0

        # bbox 기준에 도달한 뒤, 물체별 추가 전진 시간(sec)
        self.final_forward_time_by_class = {
            0: 1.0,  # can
            1: 1.0,  # bottle
            2: 1.0,  # box
        }
        self.default_final_forward_time = 0.6

        # 최종 접근 전진 속도
        self.final_forward_linear_x = 0.06
        self.final_forward_start_time = None

        # Odom 현재 위치
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # 중복 실행 방지 플래그
        self.nav_running = False
        self.backup_running = False
        self.gripper_running = False
        self.open_done = False
        self.grip_done = False
        self.release_done = False

        # Nav2 목표 재시도 관리
        self.nav_retry_count = 0
        self.max_nav_retry_count = 1
        self.last_nav_goal = None

        # BackUp 액션 완료 후 넘어갈 상태 저장
        self.backup_next_state = None

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

        self.room_index = 0

        # 물체를 발견한 방 index 저장
        self.found_room_index = None

        # 물체가 없어서 방에서 나올 때, 어느 방에서 나오는지 저장
        self.backup_from_room_index = None

        # 방에서 물체를 못 찾고 나올 때 Nav2 BackUp 설정
        # room_index 기준: Room1=0, Room2=1, Room3=2
        self.backup_before_next_room_distance_by_room = {
            0: 0.25,  # Room1에서 물체 없을 때 후진 거리
            1: 0.55,  # Room2에서 물체 없을 때 후진 거리
            2: 0.20,  # Room3에서 물체 없을 때 후진 거리
        }
        self.default_backup_before_next_room_distance = 0.25
        self.backup_before_next_room_speed = 0.08

        # Room1, Room3에서 물체를 잡은 뒤 Nav2 BackUp 설정
        self.backup_after_grip_distance_by_room = {
            0: 0.25,  # Room1에서 잡았을 때 25cm 후진
            2: 0.35,  # Room3에서 잡았을 때 35cm 후진
        }
        self.default_backup_after_grip_distance = 0.25
        self.backup_after_grip_speed = 0.08

        # 배달 장소 목표 위치
        self.delivery_goal = {
            "name": "DeliveryZone",
            "x": -0.04510348662734032,
            "y": -0.2081316113471985,
            "qz": -0.9503887257852959,
            "qw": 0.3110647358673134,
        }

        # Action client: Nav2 이동
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            self.config.navigate_to_pose
        )

        # Action client: Nav2 후진
        self.backup_client = ActionClient(
            self,
            BackUp,
            self.config.backup_action
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
        self.get_logger().info(f"backup_action: {self.config.backup_action}")
        self.get_logger().info(f"gripper_service: {self.config.gripper_service}")

    def _load_params(self):
        self.declare_parameter("hz", 10.0)

        self.declare_parameter("topic.ui", "/ui_msg")
        self.declare_parameter("topic.refiner", "/vision2taskplanner")
        self.declare_parameter("topic.detection", "/detection")
        self.declare_parameter("topic.odom", "/odom")
        self.declare_parameter("topic.cmd_vel", "/cmd_vel")

        self.declare_parameter("action.navigate_to_pose", "/navigate_to_pose")
        self.declare_parameter("action.backup", "/backup")
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

        self.config.backup_action = str(
            self.get_parameter("action.backup").value
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
            self.found_room_index = None
            self.backup_from_room_index = None
            self._clear_object_data()

            self.nav_running = False
            self.backup_running = False
            self.backup_next_state = None
            self.gripper_running = False
            self.open_done = False
            self.grip_done = False
            self.release_done = False
            self.final_forward_start_time = None
            self.nav_retry_count = 0
            self.last_nav_goal = None

            self.get_logger().info("[FSM] IDLE → DELIVER")
            self.get_logger().info("[DELIVER] Start → OpenGripper")

        elif self.putback_flag:
            self._state = "PUT_BACK"
            self.get_logger().info("[FSM] IDLE → PUT_BACK")

    def _refiner_callback(self, msg: Refiner2taskplanner):
        if msg.object_x == 0.0 and msg.object_y == 0.0:
            if self.state_deliver not in [
                "GoToObject",
                "FinalForward",
                "Grip",
                "BackupBeforeNextRoom",
                "BackupBeforeFinish",
                "IDLE_AFTER_FAIL",
                "BackupAfterGrip",
                "MoveToDelivery",
                "ReleaseObject",
                "Done",
            ]:
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

        self.last_nav_goal = {
            "x": float(x),
            "y": float(y),
            "qz": float(qz),
            "qw": float(qw),
        }

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
            self._after_nav_done()
            return

        self.get_logger().warn(
            f"Navigation failed or finished with status: {status}"
        )

        # 실패하면 같은 목표를 딱 한 번만 재전송
        if (
            self.nav_retry_count < self.max_nav_retry_count
            and self.last_nav_goal is not None
        ):
            self.nav_retry_count += 1

            self.get_logger().warn(
                f"[NAV RETRY] retry {self.nav_retry_count}/"
                f"{self.max_nav_retry_count}: "
                f"x={self.last_nav_goal['x']}, "
                f"y={self.last_nav_goal['y']}, "
                f"qz={self.last_nav_goal['qz']}, "
                f"qw={self.last_nav_goal['qw']}"
            )

            self.send_goal(
                x=self.last_nav_goal["x"],
                y=self.last_nav_goal["y"],
                qz=self.last_nav_goal["qz"],
                qw=self.last_nav_goal["qw"],
            )
            return

        # 재시도까지 실패하면 기존 FSM 흐름대로 다음 처리
        self.get_logger().error(
            "[NAV RETRY] navigation failed after one retry"
        )
        self.nav_retry_count = 0
        self._after_nav_done()

    def _after_nav_done(self):
        if self._state != "DELIVER":
            return

        self.nav_retry_count = 0

        if self.state_deliver == "RoomMove":
            if self.room_index >= len(self.rooms):
                self.get_logger().warn("[DELIVER] invalid room_index")
                self._state = "IDLE"
                return

            room_name = self.rooms[self.room_index]["name"]
            self.state_deliver = "CheckObject"
            self.get_logger().info(f"[FSM] {room_name} 이동 완료 → CheckObject")
            return

        if self.state_deliver == "MoveToDelivery":
            self.state_deliver = "ReleaseObject"
            self.get_logger().info("[FSM] 배달 장소 도착 → ReleaseObject")
            return

    def send_backup(self, distance: float, speed: float, next_state: str):
        if self.backup_running:
            return

        self.get_logger().info("Waiting for BackUp action server...")

        if not self.backup_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("BackUp action server not found")
            self.backup_running = False
            self.backup_next_state = None

            self.state_deliver = next_state
            return

        goal_msg = BackUp.Goal()

        goal_msg.target.x = -float(distance)
        goal_msg.target.y = 0.0
        goal_msg.target.z = 0.0

        goal_msg.speed = float(speed)

        goal_msg.time_allowance.sec = 5
        goal_msg.time_allowance.nanosec = 0

        self.backup_running = True
        self.backup_next_state = next_state

        self.get_logger().info(
            f"[BACKUP] request: target.x={goal_msg.target.x:.3f}, "
            f"speed={goal_msg.speed:.3f}, "
            f"next_state={next_state}"
        )

        future = self.backup_client.send_goal_async(goal_msg)
        future.add_done_callback(self._backup_goal_response_callback)

    def _backup_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"[BACKUP] goal response error: {e}")
            self.backup_running = False

            if self.backup_next_state is not None:
                self.state_deliver = self.backup_next_state

            self.backup_next_state = None
            return

        if not goal_handle.accepted:
            self.get_logger().error("[BACKUP] goal rejected")
            self.backup_running = False

            if self.backup_next_state is not None:
                self.state_deliver = self.backup_next_state

            self.backup_next_state = None
            return

        self.get_logger().info("[BACKUP] goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._backup_result_callback)

    def _backup_result_callback(self, future):
        try:
            result = future.result()
            status = result.status
        except Exception as e:
            self.get_logger().error(f"[BACKUP] result error: {e}")
            status = None

        self.backup_running = False

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("[BACKUP] succeeded")
        else:
            self.get_logger().warn(
                f"[BACKUP] failed or finished with status: {status}"
            )

        if self.backup_next_state is not None:
            next_state = self.backup_next_state
        else:
            next_state = "RoomMove"

        self.backup_next_state = None
        self.state_deliver = next_state

        self.get_logger().info(f"[FSM] BackUp 완료 → {next_state}")

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

                if self.found_room_index in [0, 2]:
                    self.state_deliver = "BackupAfterGrip"
                    self.get_logger().info(
                        f"[FSM] Grip → BackupAfterGrip "
                        f"(found_room_index={self.found_room_index})"
                    )
                    return

                self.state_deliver = "MoveToDelivery"
                self.get_logger().info("[FSM] Grip → MoveToDelivery")
                return

            if self.state_deliver == "ReleaseObject":
                self.release_done = True
                self.state_deliver = "Done"
                self.get_logger().info("[FSM] ReleaseObject → Done")
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

            case "CheckObject":
                room = self.rooms[self.room_index]
                room_name = room["name"]

                if self.object_x is not None and self.object_y is not None:
                    self.found_room_index = self.room_index

                    self.get_logger().info(
                        f"[CheckObject] {room_name}에서 물체 발견 → GoToObject"
                    )
                    self.state_deliver = "GoToObject"
                    return

                self.get_logger().warn(
                    f"[CheckObject] {room_name}에 물체 없음"
                )

                self.backup_from_room_index = self.room_index

                self.room_index += 1
                self._clear_object_data()

                if self.room_index >= len(self.rooms):
                    self.state_deliver = "BackupBeforeFinish"
                    self.get_logger().warn(
                        f"[FSM] {room_name}에 물체 없음 → BackUp 후 종료"
                    )
                    return

                next_room = self.rooms[self.room_index]["name"]

                self.state_deliver = "BackupBeforeNextRoom"
                self.get_logger().info(
                    f"[FSM] {room_name}에 물체 없음 → BackUp 후 {next_room} 이동"
                )
                return

            case "BackupBeforeNextRoom":
                self.stop_robot()

                backup_distance = self.backup_before_next_room_distance_by_room.get(
                    self.backup_from_room_index,
                    self.default_backup_before_next_room_distance
                )

                if not self.backup_running:
                    self.get_logger().info(
                        f"[BackupBeforeNextRoom] Nav2 BackUp start, "
                        f"from_room_index={self.backup_from_room_index}, "
                        f"distance={backup_distance:.3f}, "
                        f"speed={self.backup_before_next_room_speed:.3f}"
                    )

                    self.send_backup(
                        distance=backup_distance,
                        speed=self.backup_before_next_room_speed,
                        next_state="RoomMove"
                    )

                return

            case "BackupBeforeFinish":
                self.stop_robot()

                backup_distance = self.backup_before_next_room_distance_by_room.get(
                    self.backup_from_room_index,
                    self.default_backup_before_next_room_distance
                )

                if not self.backup_running:
                    self.get_logger().info(
                        f"[BackupBeforeFinish] Nav2 BackUp start, "
                        f"from_room_index={self.backup_from_room_index}, "
                        f"distance={backup_distance:.3f}, "
                        f"speed={self.backup_before_next_room_speed:.3f}"
                    )

                    self.send_backup(
                        distance=backup_distance,
                        speed=self.backup_before_next_room_speed,
                        next_state="IDLE_AFTER_FAIL"
                    )

                return

            case "IDLE_AFTER_FAIL":
                self.stop_robot()
                self.get_logger().warn("[DELIVER] 모든 방에서 물체 못 찾음")
                self._state = "IDLE"
                return

            case "GoToObject":
                IMAGE_CENTER_X = 320.0
                ALIGN_THRESH_PX = 40.0
                LINEAR_V = 0.1
                ANGULAR_V = 0.2

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
                    self.final_forward_start_time = self.get_clock().now()
                    self.state_deliver = "FinalForward"
                    self.get_logger().info(
                        f"[GoToObject] bbox_width={self.bbox_width:.1f} >= "
                        f"target={target_bbox_width:.1f} → FinalForward"
                    )
                    return

                self.get_logger().info(
                    f"[GoToObject] aligned, moving forward... "
                    f"bbox_width={self.bbox_width}, "
                    f"target={target_bbox_width}"
                )

                self.publish_cmd(LINEAR_V, 0.0)

            case "FinalForward":
                if self.final_forward_start_time is None:
                    self.final_forward_start_time = self.get_clock().now()

                now = self.get_clock().now()
                elapsed = (now - self.final_forward_start_time).nanoseconds / 1e9

                forward_time = self.final_forward_time_by_class.get(
                    self.deliver_object_id,
                    self.default_final_forward_time
                )

                if elapsed < forward_time:
                    self.publish_cmd(self.final_forward_linear_x, 0.0)
                    self.get_logger().info(
                        f"[FinalForward] class={self.deliver_object_id}, "
                        f"forward... {elapsed:.1f}/{forward_time:.1f} sec, "
                        f"linear_x={self.final_forward_linear_x:.2f}"
                    )
                    return

                self.stop_robot()
                self.final_forward_start_time = None
                self.state_deliver = "Grip"

                self.get_logger().info("[FSM] FinalForward 완료 → Grip")
                return

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

            case "BackupAfterGrip":
                self.stop_robot()

                backup_distance = self.backup_after_grip_distance_by_room.get(
                    self.found_room_index,
                    self.default_backup_after_grip_distance
                )

                if not self.backup_running:
                    self.get_logger().info(
                        f"[BackupAfterGrip] Nav2 BackUp start, "
                        f"room_index={self.found_room_index}, "
                        f"distance={backup_distance:.3f}, "
                        f"speed={self.backup_after_grip_speed:.3f}"
                    )

                    self.send_backup(
                        distance=backup_distance,
                        speed=self.backup_after_grip_speed,
                        next_state="MoveToDelivery"
                    )

                return

            case "MoveToDelivery":
                self.stop_robot()

                goal = self.delivery_goal

                self.get_logger().info(
                    f"[MoveToDelivery] Going to {goal['name']}"
                )

                self.send_goal(
                    x=goal["x"],
                    y=goal["y"],
                    qz=goal["qz"],
                    qw=goal["qw"],
                )

            case "ReleaseObject":
                self.stop_robot()

                if self.release_done:
                    return

                object_name_map = {
                    0: "can",
                    1: "bottle",
                    2: "box",
                }

                object_name = object_name_map.get(
                    self.deliver_object_id,
                    "can"
                )

                self.send_gripper(
                    object_name=object_name,
                    command=0
                )

            case "Done":
                self.stop_robot()
                self.get_logger().info("[Done] delivery completed")
                self._state = "IDLE"

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