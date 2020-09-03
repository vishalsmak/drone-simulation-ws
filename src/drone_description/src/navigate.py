#!/usr/bin/env python
import rospy
import actionlib
import tf
import roslib
import threading
import sys, time, math
import numpy as np
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from move_base_msgs.msg import MoveBaseActionFeedback

get_pos = 0

def callback(msg):
    global x, y, z,h
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.orientation.z


#odo_sub = rospy.Subscriber('/ground_truth/state', Odometry, callback)


# Get a move_base action client
rospy.init_node('patrolling')
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
rospy.loginfo('Connecting to move_base...')
client.wait_for_server()
rospy.loginfo('Connected to move_base.')

def set_goal_to_point(point):

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = point[0]
    goal.target_pose.pose.position.y = point[1]
    quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, point[2])
    goal.target_pose.pose.orientation.x = quaternion[0]
    goal.target_pose.pose.orientation.y = quaternion[1]
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()



def main():
    global x, y, z, stopp, n, m, add, yaw, a,b,c
    print"start point"
    point = (2.65,0,0)
    print "1st goal = " + str(point)
    set_goal_to_point(point)
    print"reached"
    point = (2.65,1.85,0)
    print "2nd goal = " + str(point)
    set_goal_to_point(point)
    print"reached"
    point = (2.4,1.85,0)
    print "3rd goal = " + str(point)
    set_goal_to_point(point)
    print"reached"
    point = (2.4,1,0)
    print "4th goal = " + str(point)
    set_goal_to_point(point)
    print"reached"    


if __name__ == '__main__':
    main()
    rospy.spin()
