#!/usr/bin/env python
import rospy
from gennav.envs import PolygonEnv, ScanEnv
from gennav.planners import RRT, RRG, PRM, PRMStar, PotentialField, InformedRRTstar
from gennav.utils import RobotState
from gennav.utils.geometry import Point
from gennav.utils.samplers import UniformCircularSampler, UniformRectSampler
from gennav_ros.commander import Commander
from gennav_ros.controller import Controller

planner_dict = {
    "RRT": RRT,
    "PRM": PRM,
    "PRMStar": PRMStar,
    "PotentialField": PotentialField,
    "RRG": RRG,
    "InformedRRTstar": InformedRRTstar,
}
env_dict = {"PolygonEnv": PolygonEnv, "ScanEnv": ScanEnv}


def yaml_to_params():
    param_list = rospy.get_param_names()
    param_dict = {}
    for param in param_list:
        param = param[1:]
        param_dict[param] = rospy.get_param(param)
    return param_dict


planner_name = rospy.get_param("planner_name")
env_name = rospy.get_param("env_name")
obstacles = rospy.get_param("obstacle_data")
if planner_name in planner_dict.keys():
    planner = planner_dict[planner_name]
if env_name in env_dict.keys():
    env = env_dict[env_name]
params = yaml_to_params()
planner = planner(**params)
env = env_name(**params)
env.update(obstacles)
commander = Commander(planner, env)
goal = rospy.get_param("goal")
start = rospy.get_param("start")
goal_state = RobotState(position=Point(goal[0], goal[1]))
start_state = RobotState(position=Point(start[0], start[1]))
commander.goto(goal_state, start_state)
controller = Controller()
