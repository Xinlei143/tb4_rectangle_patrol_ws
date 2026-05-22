from dataclasses import dataclass
import math


@dataclass(frozen=True)
class RectangleWaypoint:
    x: float
    y: float
    yaw: float


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def simulate_forward_offset(x, y, yaw, distance):
    return (
        x + distance * math.cos(yaw),
        y + distance * math.sin(yaw),
        yaw,
    )


def compute_rear_dock_pose(robot_x, robot_y, robot_yaw, offset, dock_yaw):
    return (
        robot_x - offset * math.cos(robot_yaw),
        robot_y - offset * math.sin(robot_yaw),
        dock_yaw,
    )


def compute_rectangle_waypoints(start_x, start_y, start_yaw, width, length):
    headings = [
        start_yaw,
        start_yaw + math.pi / 2.0,
        start_yaw + math.pi,
        start_yaw + 3.0 * math.pi / 2.0,
    ]
    distances = [width, length, width, length]

    x = start_x
    y = start_y
    waypoints = []
    for index, distance in enumerate(distances):
        heading = headings[index]
        x += distance * math.cos(heading)
        y += distance * math.sin(heading)
        next_heading = normalize_angle(headings[(index + 1) % len(headings)])
        waypoints.append(RectangleWaypoint(x=x, y=y, yaw=next_heading))

    return waypoints
