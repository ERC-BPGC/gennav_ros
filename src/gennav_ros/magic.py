#!/usr/bin/env/python
import rospy
from gennav.utils.geometry import Point as PT
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header
import math
import tf2_geometry_msgs


def convert_laser_scan_to_point(scan_data, threshold=0.25):
    """Converts data of sensor_msgs/LaserScan ROS message type to polygons
    Args:
        scan_data (sensor_msgs.msg.LaserScan): Data to be converted
        threshold (int):Threshold for deciding when to start a new obstacle
    Returns:
        list[list[tuple[float, float, float]]]: Corresponding polygons
    """
    angle_deviation = 0
    obstacle_1D = [[scan_data.ranges[0]]]
    current_obstacle_index = 0
    for i in range(len(scan_data.ranges) - 1):
        if abs(scan_data.ranges[i] - scan_data.ranges[i + 1]) > threshold:
            obstacle_1D.append([])
            current_obstacle_index += 1
        obstacle_1D[current_obstacle_index].append(scan_data.ranges[i + 1])
    obstacle_list = []
    least_angle = 2 * math.pi / len(scan_data.ranges)
    pt_count = 0

    for i in range(len(obstacle_1D)):
        for j in range(len(obstacle_1D[i])):
            obstacle_1D[i][j] = (
                obstacle_1D[i][j],
                angle_deviation + pt_count * least_angle,
            )
            pt_count = pt_count + 1

    point_list = []
    for obstacle in obstacle_1D:
        for point_rtheta in obstacle:
            point = (
                (point_rtheta[0] * math.cos(point_rtheta[1])),
                (point_rtheta[0] * math.sin(point_rtheta[1])),
                0,
            )
            point_list.append(point)
        obstacle_list.append(point_list)
    return obstacle_list


def transform_function_for_polygon_env(msg, tfBuffer, scan_frame, world_frame):
    """Transform function for gennav.envs.polygon_env
    Args:
        msg (list[list[tuple[float, float, float]]]): Corresponding polygons
        scan_frame : Valid Tf sensor frame of the robot
        world_frame : Valid Tf world frame of the robot
    """
    transform = tfBuffer.lookup_transform(world_frame, scan_frame, rospy.Time())

    new_msg = []
    for list_ in msg:
        transformed_pts = []
        for pt in list_:
            pointStamped = PointStamped(
                header=Header(frame_id=scan_frame), point=Point(pt[0], pt[1], pt[2]))
            pointTransformed = tf2_geometry_msgs.do_transform_point(
                pointStamped, transform
            )
            transformed_pts.append(
                PT(pointTransformed.point.x, pointTransformed.point.y, pointTransformed.point.z)
            )
        new_msg.append(transformed_pts)

    return new_msg
