#!/usr/bin/env python

import rospy
import tf2_geometry_msgs
import tf2_ros
from gennav.utils.geometry import Point as PT
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header


class Gennav_Transformer:
    """Base class for all the transformations"""

    def __init__(self, env, msg_dtype, world_frame, scan_frame):
        self.tfbuffer = tf2_ros.buffer()
        self.listener = tf2_ros.TransformListener(self.tfbuffer)
        # Decide on using rosparam or directly
        self.world_frame = world_frame
        self.scan_frame = scan_frame

        self.env = env
        self.msg_dtype = msg_dtype

        # registry
        self._func_registry = {(gennav.envs.PolygonEnv, sensor_msgs.msg.LaserScan): (self.convert_laser_scan_to_point, self.transform_function_for_polygon_env)}
        self.conversion, self.transform = self.get_funcs()

	def get_funcs(self):
		"""Gets conversion and transformation functions from regsitry.
	    Args:
	        env (gennav.envs.Env): Environment in consideration
	        msg_dtype (ROS message): ROS message type being considered
	    Raises:
	        NotImplementedError: If incompatible tuple is provided
	    Return:
	        (function, function): Conversion and transoform functions
	    """
	    if (self.env.__class__, self.msg_dtype.__class__) in self._func_registry.keys():
	        return self._func_registry[(self.env.__class__, self.msg_dtype.__class__)]
	    else:
	        raise NotImplementedError(
	            (self.env.__class__, self.msg_dtype.__class__), " combination is not implemented"
	        )

    def convert_laser_scan_to_point(self, scan_data, threshold = 0.25):
    	"""Converts data of sensor_msgs/LaserScan ROS message type to polygons
	    Args:
	        scan_data (sensor_msgs.msg.LaserScan): Data to be converted
	        threshold (int):Threshold for deciding when to start a new obstacle
	    Returns:
	        list[list[tuple[float, float, float]]]: Corresponding polygons
	    """
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

    def transform_function_for_polygon_env(self, msg):
        """Transform function for gennav.envs.polygon_env
        Args:
        	msg (list[list[tuple[float, float, float]]]): Corresponding polygons
        """
        transform = self.tfBuffer.lookup_transform(
            self.world_frame, self.scan_frame, rospy.Time()
        )

        new_msg = []
        for list_ in msg:
            transformed_pts = []
            for pt in list_:
                pointStamped = PointStamped(
                    header=Header(
                        frame_id=self.scan_frame, point=Point(pt[0], pt[1], pt[2])
                    )
                )
                pointTransformed = tf2_geometry_msgs.do_transform_point(
                    pointStamped, transform
                )
                transformed_pts.append(
                    PT(pointTransformed.x, pointTransformed.y, pointTransformed.z)
                )
            new_msg.append(transformed_pts)

        return new_msg

    def __call__(self, raw_msg):
    	"""Main function used for both conversion and transforming
		Args:
	        raw_msg (self.data_type) : Raw ROS Message
	    Returns:
	        list[list[gennav.utils.geometry.Point]]: Corresponding polygons
    	"""
   		return self.transform(self.conversion(raw_msg))