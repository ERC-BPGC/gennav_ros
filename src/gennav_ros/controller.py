import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectory

from gennav_ros.conversions import Odom_to_RobotState, Velocity_to_Twist, msg_to_traj


class Controller:
    def __init__(self, controller):
        rospy.init_node("controller", anonymous=True)
        self._traj_sub = rospy.Subscriber(
            "/gennav/traj", MultiDOFJointTrajectory, self._traj_cb
        )
        self._vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self._odom_cb)
        self.traj = None
        self.controller = controller
        self.velocities = Twist()
        self.timer = rospy.Timer(self._publish_vel)

    def _traj_cb(self, msg):
        """Callback function for trajectory.
        Args:
            msg (MultiDOFJointTrajectory): Subscribed Trajectory on /gennav/traj topic
        """
        self.traj = msg_to_traj(msg)

    def _odom_cb(self, msg):
        """Callback function for odometry
        Args:
            msg (Odometry): Subscribed Odometry of the robot on /odom topic
        """
        self.controller.set_state(Odom_to_RobotState(msg))

    def _publish_vel(self):
        """Method to publish the velocities on /cmd_vel topic.
        """
        if self.traj is None:
            pass
        else:
            self.velocities = Velocity_to_Twist(self.controller.compute_vel(self.traj))
            self._vel_pub.publish(self.velocities)