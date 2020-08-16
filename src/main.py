#!/usr/bin/env python
import rospy
from gennav.envs import *
from gennav.planners import *
from gennav.utils import RobotState
from gennav.utils.geometry import Point
from gennav.utils.samplers import *
from gennav_ros.commander import Commander
from gennav_ros.controller import Controller

planner_name = rospy.get_param("planner_name")
env_name = rospy.get_param("env_name")
obstacles = rospy.get_param("obstacle_data")
planner = eval(planner_name)
env = eval(env_name)
env.update(obstacles)
commander = Commander(planner, env)
goal = rospy.get_param("goal")
start = rospy.get_param("start")
goal_state = RobotState(position=Point(goal[0], goal[1]))
start_state = RobotState(position=Point(start[0], start[1]))
commander.goto(goal_state, start_state)
controller = Controller()
