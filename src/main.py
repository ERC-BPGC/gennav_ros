#!/usr/bin/env python
import rospy
from gennav_ros.commander import Commander
from gennav_ros.controller import Controller

planner=rospy.get_param("planner")
env_name=rospy.get_param("env_name")
goal=rospy.get_param("goal")
start=rospy.get_param("start")
commander=Commander(planner,env_name)
commander.goto(goal,start)
controller=Controller()