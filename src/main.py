#!/usr/bin/env python
import rospy
from gennav.envs import *
from gennav.planners import *
from gennav.utils import RobotState
from gennav.utils.geometry import Point
from gennav.utils.samplers import *
from gennav_ros.commander import Commander
from gennav_ros.controller import Controller

def planner_args():
    param_list=rospy.get_param_names()
    for param in param_list:
        param_list[param_list.index(param)]=param[1:]
    param_dict={}
    chk=eval(planner_name+".__init__.__code__.co_varnames")
    for param in param_list:
        if param in chk:
            param_dict[param]=rospy.get_param(param)
    return param_dict

def env_args():
    param_list=rospy.get_param_names()
    for param in param_list:
        param_list[param_list.index(param)]=param[1:]
    print param_list
    param_dict={}
    chk=eval(env_name+".__init__.__code__.co_varnames")
    for param in param_list:
        if param in chk:
            param_dict[param]=rospy.get_param(param)
    return param_dict


planner_name = rospy.get_param("planner_name")
env_name = rospy.get_param("env_name")
obstacles = rospy.get_param("obstacle_data")
planner_params=planner_args()
env_params=env_args()
planner = eval(planner_name+"(**planner_params)")
env = eval(env_name+"(**env_params)")
env.update(obstacles)
commander = Commander(planner, env)
goal = rospy.get_param("goal")
start = rospy.get_param("start")
goal_state = RobotState(position=Point(goal[0], goal[1]))
start_state = RobotState(position=Point(start[0], start[1]))
commander.goto(goal_state, start_state)
controller = Controller()