import gennav
import sensor_msgs
from gennav_ros import conversions


def transform_polygons(polygons, position):
    """Transforms polygons with respect to some position

    Args:
        polygons (list[list[tuple[float, float, float]]]): polygon
            to be transformed
        position (gennav.utils.RobotState): position w.r.t. which
            polygons need to be transformed

    Returns:
    list[list[tuple[float, float, float]]]: Transformed polygons
    """
    raise NotImplementedError


_func_registry = {
    (gennav.envs.PolygonEnv, sensor_msgs.msg.LaserScan): (
        conversions.LaserScan_to_polygons,
        transform_polygons,
    )
}


def get_funcs(env, msg_dtype):
    """Gets conversion and transformation functions from regsitry.

    Args:
        env (gennav.envs.Env): Environment in consideration
        msg_dtype (ROS message): ROS message type being considered

    Raises:
        NotImplementedError: If incompatible tuple is provided

    Return:
        (function, function): Conversion and transoform functions
    """
    if (env.__class__, msg_dtype.__class__) in _func_registry.keys():
        return _func_registry[(env.__class__, msg_dtype.__class__)]
    else:
        raise NotImplementedError(
            (env.__class__, msg_dtype.__class__), " combination is not implemented"
        )
