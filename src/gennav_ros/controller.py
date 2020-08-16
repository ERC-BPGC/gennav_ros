import gennav
import rospy
from gennav_ros.conversions import msg_to_traj
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import MultiDOFJointTrajectory


class Controller:
    def __init__(self):
        self._traj_sub = rospy.Subscriber(
            "/gennav/traj", MultiDOFJointTrajectory, callback=self._traj_cb
        )
        self._vel_pub = rospy.Publisher("/cmd_vel", Twist)
        self.traj = None

        timer = rospy.Timer(self._publish_vel)

    def _traj_cb(self, msg):
        self.traj = msg_to_traj(msg)

    def _publish_vel(self):
        if self.traj is None:
            pass
        else:
            raise NotImplementedError
