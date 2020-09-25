#!/usr/bin/env python

import rospy
import sensor_msgs

import gennav
from gennav.utils import RobotState
from gennav.utils.geometry import Point

import gennav_ros

# Dictionary of implemented samplers
sampler_registry = {
    "UniformRectSampler": gennav.utils.samplers.UniformRectSampler,
    "UniformCircularSampler": gennav.utils.samplers.UniformCircularSampler,
}

# Dictionary of implemented planners
planner_registry = {
    "RRT": gennav.planners.RRT,
    "PRM": gennav.planners.PRM,
    "PRMStar": gennav.planners.PRMStar,
    "PotentialField": gennav.planners.PotentialField,
    "RRG": gennav.planners.RRG,
    "InformedRRTstar": gennav.planners.InformedRRTstar,
}

# Dictionary of implemented environments
env_registry = {"PolygonEnv": gennav.envs.PolygonEnv, "ScanEnv": gennav.envs.ScanEnv}

# Dictionary of implemented controllers
controller_registry = {
    "DIffPID": gennav.controllers.DIffPID,
    "OmniWheelPID": gennav.controllers.OmniWheelPID,
}

# Dictionary of compatible ROS message types
msg_dtype_registry = {"sensor_msgs/LaserScan": sensor_msgs.msg.LaserScan}

# Get a dictionary of parameters from ROS parameter server
param_list = rospy.get_param_names()
param_dict = {}
for param in param_list:
    param = param[1:]
    param_dict[param] = rospy.get_param(param)

# Get start and goal points
goal = param_dict["goal"]
start = param_dict["start"]

# Instantiate sampler
sampler_name = param_dict["sampler_name"]
if sampler_name in sampler_registry.keys():
    sampler = sampler_registry[sampler_name](**param_dict)
else:
    raise NotImplementedError("Specified sampler ", sampler_name, " is not implemented")

# Instantiate planner
planner_name = param_dict["planner_name"]
if planner_name in planner_registry.keys():
    planner = planner_registry[planner_name](sampler=sampler, **param_dict)
else:
    raise NotImplementedError("Specified planner ", planner_name, " is not implemented")

# Instantiate environemnt
env_name = param_dict["env_name"]
if env_name in env_registry.keys():
    env = env_registry[env_name](**param_dict)
    env.update(param_dict["obstacle_data"])
else:
    raise NotImplementedError("Specified environment ", env_name, " is not implemented")

# Instantiate controller
controller_name = param_dict["controller_name"]
if controller_name in controller_registry.keys():
    controller = controller_registry[controller_name](**param_dict)
else:
    raise NotImplementedError("Specified controller ", env_name, " is not implemented")

# Get message type object
msg_dtype_name = param_dict["msg_dtype_name"]
if msg_dtype_name in msg_dtype_registry.keys():
    msg_dtype = msg_dtype_registry[msg_dtype_name]()
else:
    raise NotImplementedError(
        "Specified message type ", planner_name, " is not compatible"
    )

# Construct controller
controller_node = gennav_ros.Controller(controller)

# Construct  commander
replan_interval = rospy.Duration(param_dict["replan_interval"])
commander_node = gennav_ros.Commander(
    planner=planner, env=env, replan_interval=replan_interval, msg_dtype=msg_dtype,
)

# Execute command
goal_state = RobotState(position=Point(*goal))
start_state = RobotState(position=Point(*start))
commander_node.goto(goal_state, start_state)
