#!/usr/bin/env python

import rospy
import tf2_ros
from tf.transformations import euler_from_quaternion
from helper_funtions.drone_3d_trajectory_following import *

global flag_in


# this file is a crude path planning code which can be executed as well

def trajectory_calculation(start_point, end_point):
    x_coeffs = [[]]
    y_coeffs = [[]]
    z_coeffs = [[]]
    T = 4
    traj = TrajectoryGenerator(start_point, end_point, T)
    traj.solve()
    x_coeffs[0] = traj.x_c
    y_coeffs[0] = traj.y_c
    z_coeffs[0] = traj.z_c

    quad_run_simulate(x_coeffs, y_coeffs, z_coeffs, start_point[0], start_point[1], start_point[2])


if __name__ == '__main__':
    global flag_in
    rospy.init_node('plot_path_node')

    # initialising tf listener
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # setting up rate at which to transmit data and PID rate
    rate = rospy.Rate(10.0)
    marker_found_flag = False
    is_marker_there = True
    state = dict()

    set_point_array = np.array([0.0, 0.0, 0.0])
    while not rospy.is_shutdown():

        # If the marker is visible switch to land loop
        try:
            trans = tfBuffer.lookup_transform('marker_frame', 'base_link', rospy.Time()).transform
            (euler_r, euler_p, euler_y) = euler_from_quaternion(
                [trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w])
            current_pose = np.array([trans.translation.x, trans.translation.y, trans.translation.z])

            print('marker found')
            marker_found_flag = True
            flag_in = True

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):

            rate.sleep()
            continue

        if marker_found_flag and is_marker_there:
            marker_found_flag = False
            is_marker_there = False
            trajectory_calculation(current_pose, set_point_array)
