import numpy as np
from pydrake.all import PiecewisePolynomial
from trajectory_msgs.msg import JointTrajectory
# from rclpy.time import Time
from builtin_interfaces.msg import Time


def time_msg_to_float(time:Time):
    return time.sec + time.nanosec * 1e-9

def duration_msg_to_float(duration):
    return duration.sec + duration.nanosec * 1e-9

def convert_ros_traj_to_drake_traj(traj:JointTrajectory):
    num_steps = len(traj.points)
    assert num_steps > 0
    nq = len(traj.joint_names)

    q_knots = np.zeros((num_steps, nq))
    for i in range(num_steps):
        q_knots[i] = traj.points[i].positions

    t_knots = time_msg_to_float(traj.header.stamp) + \
        np.array([duration_msg_to_float(traj.points[i].time_from_start) for i in range(num_steps)])
    
    q_polynomial = PiecewisePolynomial.CubicWithContinuousSecondDerivatives(t_knots, q_knots.T)

    return q_polynomial
