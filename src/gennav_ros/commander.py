import gennav
import rospy
from gennav_ros.utils import parse_env_str
from trajectory_msgs.msg import MultiDOFJointTrajectory
from gennav_ros.conversions import traj_to_msg
class Commander:
    def __init__(self, planner, env_name):
        self.planner = planner
        self.env, self.transform_env_data, self.msg_to_env_data = parse_env_str(env_name)

        self._env_sub = rospy.Subscriber("/gennav/env", callback=self._env_cb)
        self._traj_pub = rospy.Publisher("/gennav/traj", )

    def goto(self, goal, start=None):
        traj = self.planner.plan(self.curr_state, goal)
        self._publish_traj(traj)

        def replan():
            if not self.env.get_traj_status(traj):
                traj = self.planner.replan(self.curr_state, goal)
                self._publish_traj(traj)
        
        timer = rospy.Timer(replan)

    def _publish_traj(self, traj):
        traj_msg = traj_to_msg(traj)
        self._traj_pub.publish(traj_msg)

    def _env_cb(self, msg):
        data = self.msg_to_env_data(msg)
        data = self.transform_env_data(data)
        self.env.update(data)
