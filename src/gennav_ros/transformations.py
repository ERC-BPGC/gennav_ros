#!/usr/bin/env python

import rospy
import tf2_ros
from gennav.envs import polygon_env
from gennav_ros.magic import (
    convert_laser_scan_to_point,
    transform_function_for_polygon_env,
)
from sensor_msgs.msg import LaserScan


class Transformer:
    """Base class for all the transformations
    Args:
        env (gennav.envs) : Gennav Env for environment update
        msg_dtype (ROS Message) : Raw ROS Message for further gennav conversions
    """

    def __init__(self, env, msg_dtype, *args ,**kwargs):
        self.tfbuffer = None
        self.listener = None
        self.env = env
        self.msg_dtype = msg_dtype

        self.kwargs = kwargs

        print self.env
        print self.msg_dtype

    def init(self):
        """Init method for initialisng valid TF buffer and TF Listener
        """
        self.tfbuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfbuffer)

    def __call__(self, raw_msg):
        """Main function used for both conversion and transforming
        Args:
            raw_msg (self.msg_dtype) : Raw ROS Message
        Returns:
            list[list[gennav.utils.geometry.Point]]: Corresponding polygons
        """
        if self.tfbuffer is None or self.listener is None:
            raise RuntimeError("Transform Buffer and Listener not Initialised")
        elif (
            self.env.__class__ == polygon_env.PolygonEnv
            and self.msg_dtype == LaserScan
        ):
            return transform_function_for_polygon_env(
                convert_laser_scan_to_point(raw_msg), self.tfbuffer,
                "base_scan", "world",
            )
        else:
            raise NotImplementedError("Other Functionalites yet to be Implemented")
