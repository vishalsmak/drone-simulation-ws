#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import *
from geometry_msgs.msg import *
from tf.transformations import euler_from_quaternion
import tf2_ros
from helper_funtions.pid import pid
import numpy as np
import time

if __name__ == '__main__':
    rospy.init_node('aruco_land')
    # initialising tf listener
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # setting up rate at which to transmit data
    rate = rospy.Rate(10.0)
    found = False

    state = dict()
    # initialising variables for PID
    state['lastError'] = np.array([0., 0., 0., 0.])

    xy_pid_bottom = [2, 0., 0.2]
    state['p'] = np.array(
        [xy_pid_bottom[0], xy_pid_bottom[0], 1, 0.5], dtype=float)
    state['i'] = np.array(
        [xy_pid_bottom[1], xy_pid_bottom[1], 0.1, 0.1], dtype=float)
    state['d'] = np.array(
        [xy_pid_bottom[2], xy_pid_bottom[2], 1, 0.05], dtype=float)

    # initialising variables for PID
    state['integral'] = np.array([0., 0., 0., 0.])
    state['derivative'] = np.array([0., 0., 0., 0.])

    state['last_time'] = time.time()

    # Set this to the waypoint you want to travel to
    set_array = np.array([0., 0., 4, 0.])
    i = 0

    # stay in loop unless rospy is closed
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('marker_frame', 'bottom_optical_frame', rospy.Time()).transform
            (euler_r, euler_p, euler_y) = euler_from_quaternion([trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w])

            current_pose = np.array([trans.translation.x, trans.translation.y, trans.translation.z, euler_y])

            pid_twist, state = pid(current_pose, set_array, state)
            if state['lastError'][0] < 0.5 and state['lastError'][1] < 0.5:
                i += 1
            else:
                i = 0

            if i >= 10:
                set_array = np.array([0., 0., 0, 0.])

            # vel = Twist()
            # vel.linear.x = pid_twist[0]
            # vel.linear.y = pid_twist[1]
            # vel.linear.z = pid_twist[2]
            # vel.angular.z = pid_twist[3]

            velocity_publisher.publish(pid_twist)
            found = True

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            vel = Twist()
            vel.linear.x = 0
            vel.linear.y = 0
            vel.linear.z = 0 if found else 0.5
            vel.angular.z = 0 #yaw
            velocity_publisher.publish(vel)
            print('not yet found')
            continue
        # sleep for the time in rate
        rate.sleep()