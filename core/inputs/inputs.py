import math

from nav_msgs.msg import Odometry
from turtlebot_interfaces.msg import Refiner2taskplanner
from client_vision_interfaces.msg import TurtlebotDetection
from ..blackboard.blackboard import bb


def quaternion_to_yaw(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def odom_callback(msg: Odometry):
    bb.robot_x = msg.pose.pose.position.x
    bb.robot_y = msg.pose.pose.position.y
    bb.robot_yaw = quaternion_to_yaw(msg.pose.pose.orientation)


def vision2taskplanner_callback(msg: TurtlebotDetection):
    bb.has_detection = len(msg.class_ids) > 0
    bb.detection = msg if bb.has_detection else None


def refiner_callback(msg: Refiner2taskplanner):
    bb.object_dist = msg.object_dist
    bb.object_theta = msg.object_theta