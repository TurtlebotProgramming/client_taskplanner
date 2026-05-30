import math

DEG2RAD = math.pi / 180.0


def object_to_world_pos(robot_x: float, robot_y: float, robot_yaw: float,
                        dist: float, theta_deg: float):
   
    angle = robot_yaw + theta_deg * DEG2RAD
    return robot_x + dist * math.cos(angle), robot_y + dist * math.sin(angle)
