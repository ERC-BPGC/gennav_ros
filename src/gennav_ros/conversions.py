import tf
from geometry_msgs.msg import Point, Quaternion, Transform, Twist

import gennav.utils as utils
from gennav.utils import RobotState, Trajectory, Velocity
from tajectory_msgs.msg import (
    MultiDOFJointTrajectory,
    MultiDOFJointTrajectoryPoint,
)


def traj_to_msg(traj):
    traj_msg = MultiDOFJointTrajectory(
        points=[], joint_names=None, header=None
    )
    for state, timestamp in zip(traj.path, traj.timestamps):
        quaternion = tf.transformations.quaternion_from_euler(
            state.orientation.roll,
            state.orientation.pitch,
            state.orientation.yaw,
        )
        velocity = Twist()
        acceleration = Twist()
        transforms = Transform(
            translation=Point(
                state.position.x, state.position.y, state.position.z
            ),
            rotation=Quaternion(
                quaternion[0], quaternion[1], quaternion[2], quaternion[3]
            ),
        )
        traj_point = MultiDOFJointTrajectoryPoint(
            transforms=[transforms],
            velocities=[velocity],
            accelerations=[acceleration],
            time_from_start=timestamp,
        )
        traj_msg.points.append(traj_point)

    return traj_msg


def msg_to_traj(msg):
    path = []
    timestamps = []
    for point in msg.points:
        timestamps.append(point.time_from_start)
        position = utils.Point(
            x=point.transforms[0].translation.x,
            y=point.transforms[0].translation.y,
            z=point.transforms[0].translation.z,
        )
        rotation = tf.transformations.euler_from_quaternion(
            [
                point.transforms[0].rotation.x,
                point.transforms[0].rotation.y,
                point.transforms[0].rotation.z,
                point.transforms[0].rotation.w,
            ]
        )
        rotation = utils.OrientationRPY(
            roll=rotation[0], pitch=rotation[1], yaw=rotation[2]
        )
        linear_vel = utils.Vector3D(
            x=point.velocities[0].linear.x,
            y=point.velocities[0].linear.y,
            z=point.velocities[0].linear.z,
        )
        angular_vel = utils.Vector3D(
            x=point.velocities[0].angular.x,
            y=point.velocities[0].angular.y,
            z=point.velocities[0].linear.z,
        )
        velocity = Velocity(linear=linear_vel, angular=angular_vel)
        state = RobotState(
            position=position, orientation=rotation, velocity=velocity
        )
        path.append(state)

    traj = Trajectory(path=path, timestamps=timestamps)
    return traj
