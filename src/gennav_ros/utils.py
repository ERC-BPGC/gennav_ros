import gennav

from gennav_ros.conversions import qW
from sensor_msgs.msg import LaserScan


_env_registry = {gennav.envs.PolygonEnv: ()}


def parse_env(env):
    return _env_registry[env]


_dtype_registry = {"laserscan": LaserScan}


def parse_dtype(msg_dtype_str):
    return _dtype_registry[msg_dtype_str]
