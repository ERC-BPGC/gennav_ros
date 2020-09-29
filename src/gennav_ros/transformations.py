#!/usr/bin/env python

import rospy
import tf2_geometry_msgs
import tf2_ros
from gennav.utils.geometry import Point as PT
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header


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
        msg : list of lists of gennav.geometry.common.Point
        """
        transform = cls.tfBuffer.lookup_transform(
            cls.world_frame, cls.scan_frame, rospy.Time()
        )

        new_msg = []
        for list_ in msg:
            transformed_pts = []
            for pt in list_:
                pointStamped = PointStamped(
                    header=Header(
                        frame_id=cls.scan_frame, point=Point(pt.x, pt.y, pt.z)
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
