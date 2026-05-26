import math

from nav_msgs.msg import Odometry
from turtlebot_interfaces.msg import TurtlebotDetection
from ..blackboard.blackboard import bb


def odom_callback(msg: Odometry):
    bb.robot_x = msg.pose.pose.position.x
    bb.robot_y = msg.pose.pose.position.y
    q = msg.pose.pose.orientation
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    bb.robot_yaw = math.atan2(siny, cosy)


def refiner_callback(msg: TurtlebotDetection):
    bb.has_detection = len(msg.class_ids) > 0
    bb.detection = msg if bb.has_detection else None


    ## 서비스로 하면 되겠다 path planner랑 