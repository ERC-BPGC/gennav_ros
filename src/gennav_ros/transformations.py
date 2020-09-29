#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import PointStamped
from gennav.utils.geometry import Point as PT
from std_msgs.msg import Header
import tf2_geometry_msgs


class Gennav_Transformer:
	"""Base class for all the transformations"""
	def __init__(self):
		self.tfbuffer = tf2_ros.buffer()
		self.listener = tf2_ros.TransformListener(self.tfbuffer)
		self.world_frame = rospy.get_param("world_frame")
		self.scan_frame = rospy.get_param("robot_frame")

	@classmethod
	def transform_function_for_polygon_env(cls, msg):
		"""Transform function for gennav.envs.polygon_env
		Args:
			msg([[gennav.common.Point]]) : A list of lists of gennav.utils.geometry.Point
		Returns:
			A list of lists of transformed gennav.utils.geometry.Point
		"""
		transform = self.tfBuffer.lookup_transform(self.world_frame, self.scan_frame, rospy.Time())

		new_msg = []
		for list_ in msg:
			transformed_pts = []
			for pt in list_ :
				pointStamped = PointStamped(header=Header(frame_id=self.scan_frame, point = Point(pt.x, pt.y, pt.z)))
				pointTransformed = tf2_geometry_msgs.do_transform_point(pointStamped, transform)
				transformed_pts.append(PT(pointTransformed.x, pointTransformed.y, pointTransformed.z))
			new_msg.append(transformed_pts)

		return new_msg