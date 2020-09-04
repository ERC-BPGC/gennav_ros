import gennav
import rospy
from gennav_ros.conversions import Odom_to_RobotState, traj_to_msg
from gennav_ros.utils import parse_env
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from trajectory_msgs.msg import MultiDOFJointTrajectory


class Commander:
    def __init__(self, planner, env, duration):
        """Init Commander Parameters.

        Args:
            planner(gennav.planners.Planner): object of the Planner class for path planning algorithms
            env(gennav.envs.Environment): object of the Environment class for an environment
            duration(rospy.Duration):  desired period between callbacks
        """
        rospy.init_node("commander", anonymous=True)
        self.planner = planner
        (
            self.env,
            self.transform_env_data,
            self.msg_to_env_data,
            self.data_type,
        ) = parse_env(env)

        self.replan_int = duration
        self.curr_state = gennav.utils.RobotState()
        self._env_sub = rospy.Subscriber(
            "/gennav/env", LaserScan, callback=self._env_cb
        )
        self._odom_sub = rospy.Subscriber("/odom", Odometry, self._odom_cb)
        self._traj_pub = rospy.Publisher(
            "/gennav/traj", MultiDOFJointTrajectory, queue_size=10
        )
        self.timer = rospy.Timer()
        self.traj = gennav.utils.Trajectory()

    def goto(self, goal, start=None):
        """Method to find Trajectory of the robot using the planner.plan method.

        Args:
            goal (gennav.utils.RobotState): A RobotState() object which is passed to the planner.plan method.
            start (gennav.utils.RobotState, optional): A RobotState() object which can be passed to the planner.plan method. Defaults to None.
        """
        if start is not None:
            self.traj = self.planner.plan(start, goal, self.env)
        else:
            self.traj = self.planner.plan(self.curr_state, goal, self.env)
        self._publish_traj(self.traj)

        def replan():
            """Method to plan the path again using the planner.replan method."""
            if not self.env.get_traj_status(self.traj):
                traj = self.planner.replan(self.curr_state, goal, self.env)
                self._publish_traj(traj)

        self.timer = rospy.Timer(rospy.Duration(self.replan_int), replan)

    def _publish_traj(self, traj):
        """Method to publish the velocities on /cmd_vel topic.

        Args:
            traj (gennav.utils.Trajectory): Trajectory planned by the planner
        """
        traj_msg = traj_to_msg(traj)
        self._traj_pub.publish(traj_msg)

    def _env_cb(self, msg):
        """Callback function for environment

        Args:
            msg (sensor_msgs/LaserScan.msg): Subscribed to LaserScan (ROS msg) data of the robot on /gennav/env topic
        """
        data = self.msg_to_env_data(msg)
        data = self.transform_env_data(data, self.curr_state)
        self.env.update(data)

    def _odom_cb(self, msg):
        """Callback function for odometry

        Args:
            msg (Odometry): Subscribed Odometry of the robot on /odom topic
        """
        self.curr_state = Odom_to_RobotState(msg)
