#!/usr/bin/env python

import rospy
import tf2_ros
import numpy as np
import time
from geometry_msgs.msg import *
from tf.transformations import euler_from_quaternion
from helper_funtions.pid import pid

if __name__ == '__main__':

    rospy.init_node('drone_takeoff_land')

    # initialising tf listener
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # setting up rate at which to transmit data and PID rate
    rate = rospy.Rate(10.0)
    marker_found_flag = False
    state = dict()

    # initialising variables for PID

    translation_pid_factors = [0.8, 0.001, 0.5]
    state['p'] = np.array([translation_pid_factors[0], translation_pid_factors[0], translation_pid_factors[0]],
                          dtype=float)
    state['i'] = np.array([translation_pid_factors[1], translation_pid_factors[1], translation_pid_factors[1]],
                          dtype=float)
    state['d'] = np.array([translation_pid_factors[2], translation_pid_factors[2], translation_pid_factors[2]],
                          dtype=float)

    # initialising variables for PID
    state['lastError'] = np.zeros(3)
    state['integral'] = np.zeros(3)
    state['derivative'] = np.zeros(3)
    state['last_time'] = time.time()

    # Set this to the waypoint you want to travel to so in this case its origin because we are approaching marker
    # frame (0 , 0, 0)
    set_point_array = np.array([0.0, 0.0, 0.0])
    # stay in loop unless rospy is closed
    while not rospy.is_shutdown():

        # If the marker is visible switch to land loop
        try:
            trans = tfBuffer.lookup_transform('marker_frame', 'base_link', rospy.Time()).transform
            (euler_r, euler_p, euler_y) = euler_from_quaternion(
                [trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w])
            current_pose = np.array([trans.translation.x, trans.translation.y, trans.translation.z])
            output, state = pid(current_pose, set_point_array, state)

            pid_twist = Twist()
            pid_twist.linear.x = output[0]
            pid_twist.linear.y = output[1]
            pid_twist.linear.z = -output[2]
            pid_twist.angular.z = 0

            velocity_publisher.publish(pid_twist)
            print('marker found')
            marker_found_flag = True

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):

            if not marker_found_flag:
                vel = Twist()
                vel.linear.x = 0
                vel.linear.y = 0
                vel.linear.z = 0.5
                vel.angular.z = 0   #yaw
                velocity_publisher.publish(vel)
                print('marker not yet found')
                continue
        # sleep for the time in rate
        rate.sleep()