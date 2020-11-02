import gennav
import rospy
from gennav_ros.conversions import Odom_to_RobotState, traj_to_msg
from gennav_ros.utils import get_funcs
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectory
from gennav_ros.transformations import Transformer


class Commander:
    """Coordinates planning and control

    Args:
        planner (gennav.planners.Planner): object of the Planner class for path planning algorithms
        env (gennav.envs.Environment): object of the Environment class for an environment
        msg_dtype (ROS Message): Type of message used for envrionment data
        replan_interval (rospy.Duration): Desired period between callbacks. Defaults to 1s
    """

    def __init__(self, planner, env, msg_dtype, replan_interval=rospy.Duration(1)):
        # Initialise commander node
        #rospy.init_node("commander", anonymous=True)

        # Store attributes
        self.planner = planner
        self.env = env
        self.replan_interval = replan_interval
        self.msg_dtype = msg_dtype

        # Intialise the transformer for obtaining transforms
        self.transformer = Transformer(self.env, self.msg_dtype)
        # init method of the transformer sets a valid tf buffer and listener
        self.transformer.init()

        # Initialise variables
        self.traj = gennav.utils.Trajectory()
        self.curr_state = gennav.utils.RobotState()

        # Subscribe to envrionment data topic
        self._env_sub = rospy.Subscriber(
            "/scan", self.msg_dtype, callback=self._env_cb
        )

        # Subscribe to odometry
        self._odom_sub = rospy.Subscriber("/odom", Odometry, self._odom_cb)

        # Publisher for planned trajectory
        self._traj_pub = rospy.Publisher(
            "/gennav/traj", MultiDOFJointTrajectory, queue_size=10
        )
        

    def goto(self, goal, start=None):
        """Method to find Trajectory of the robot using the planner.plan method.
        Args:
            goal (gennav.utils.RobotState): A RobotState() object which is passed to the planner.plan method.
            start (gennav.utils.RobotState, optional): A RobotState() object which can be passed to the planner.plan method. Defaults to None.
        """
        # Set current state as start if not specified
        start = self.curr_state if start is not None else start

        # Compute first planned trajectory
        self.traj = self.planner.plan(start, goal, self.env)

        # Publish first planned trajectory
        self._publish_traj(self.traj)

        def replan():
            """Method to plan the path again using the planner.replan method."""
            if not self.env.get_traj_status(self.traj):
                self.traj = self.planner.replan(self.curr_state, goal, self.env)
                self._publish_traj(self.traj)

        # Keep replanning at specified intervals
        self.timer = rospy.Timer(rospy.Duration(self.replan_interval), replan)

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
            msg (self.msg_dtype): Subscribed to ROS msg data of the robot on /gennav/env topic
        """
        print "data recieved"
        data = self.transformer(msg)
        print "transformed"        
        self.env.update(data)

    def _odom_cb(self, msg):
        """Callback function for odometry
        Args:
            msg (Odometry): Subscribed Odometry of the robot on /odom topic
        """
        self.curr_state = Odom_to_RobotState(msg)
