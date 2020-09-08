#!/usr/bin/env python
"""Generic PID implimentation. vishal 12/07/19"""
import rospy
from std_msgs.msg import Float64
import numpy as np
import time

pubx = rospy.Publisher('plot_x', Float64, queue_size=5)
puby = rospy.Publisher('plot_y', Float64, queue_size=5)
pubz = rospy.Publisher('plot_z', Float64, queue_size=5)
pub_yaw = rospy.Publisher('plot_yaw', Float64, queue_size=5)


def pid(current_data_array, set_point_array, state):
    """Run one step of PID.

    convention in which to send data:
    data[0] = x-axis (+ve is front)
    data[1] = y-axis (-ve is left)
    data[2] = z-axis (+ve is up)
    data[3] = yaw (+ve is counter-clockwise

    convention in which to send data for aruco_mapping:
    data[0] = x-axis (positive values always and front is less is front)
    data[1] = y-axis (-ve is left)
    data[2] = z-axis (+ve is up)
    data[3] = yaw (+ve is counter-clockwise

    Args:
        current_data_array (numpy.array): current pose of the drone
        set_point_array (numpy.array): pose to attain
        state (dict): storing all the values to be used in every call to pid

    Returns:
        geometry_msgs.msg.Twist: twist to be published to cmd_vel for drone
        dict: storing all the values to be used in every call to pid
    """
    current_time = time.time()
    dt = current_time - state['last_time']

    error = current_data_array - set_point_array
    error = np.around(error, decimals=2)

    # publish errors for plotting.
    pubx.publish(error[0])
    puby.publish(error[1])
    pubz.publish(error[2])
    # error_pub.publish(error)

    state['integral'] += error * dt
    state['derivative'] = (error - state['lastError']) / dt

    output = state['p'] * error + state['i'] * \
             state['integral'] + state['d'] * state['derivative']

    state['lastError'] = error
    state['last_time'] = current_time

    output = np.clip(output, -0.4, 0.4)

    return output, state
