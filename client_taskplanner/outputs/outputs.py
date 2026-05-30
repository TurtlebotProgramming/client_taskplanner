from nav2_msgs.action import NavigateToPose
from turtlebot_interfaces.srv import Gripper


def send_nav_goal(nav_client, logger, x: float, y: float, done_cb=None):
    if not nav_client.server_is_ready():
        logger.warn("[NAV] 서버 없음 - 대기 중")
        return
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.pose.position.x = x
    goal.pose.pose.position.y = y
    goal.pose.pose.orientation.w = 1.0
    future = nav_client.send_goal_async(goal)
    if done_cb is not None:
        def _on_goal(gf):
            handle = gf.result()
            if not handle.accepted:
                logger.warn("[NAV] 목표 거부됨")
                done_cb(False)
                return
            handle.get_result_async().add_done_callback(
                lambda _: done_cb(True)
            )
        future.add_done_callback(_on_goal)


def send_gripper(gripper_client, logger, command: int, done_cb=None):
    if not gripper_client.service_is_ready():
        logger.warn("[GRIPPER] 서비스 없음")
        if done_cb is not None:
            done_cb(False)
        return
    req = Gripper.Request()
    req.command = command
    future = gripper_client.call_async(req)
    if done_cb is not None:
        def _on_gripper(f):
            try:
                done_cb(f.result().success)
            except Exception as e:
                logger.error(f"[GRIPPER] 서비스 응답 오류: {e}")
                done_cb(False)
        future.add_done_callback(_on_gripper)
