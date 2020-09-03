#!/usr/bin/env python
from mpl_toolkits import mplot3d
import time
import roslib
import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from nav_msgs.msg import Odometry
from decimal import Decimal

def callback(msg):
    global x,y,z,a,b,c
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z

odo_sub = rospy.Subscriber('/ground_truth/state', Odometry, callback)
rospy.init_node('trajectory_graph')

z_points =list()
x_points =list()
y_points =list()

fig = plt.figure()
ax = plt.axes(projection="3d")

while not rospy.is_shutdown():
    a = round(x,2)
    b = round(y,2)
    c = round(z,2)
    print str(a)+" "+str(b)+" "+str(c)
    x_points.append(a)
    y_points.append(b)
    z_points.append(c)
    ax.plot(x_points, y_points, z_points)
    plt.pause(0.01)
