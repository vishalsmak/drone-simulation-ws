#!/usr/bin/env python
import time
import roslib
import rospy
from matplotlib import pyplot as plt
import matplotlib.animation as animation
from gazebo_msgs.msg import ContactsState
from decimal import Decimal


def callback(msg):
    global x,y,z,a,b,c
    x = msg.states[0].contact_positions[0].x
    print x
    # y = msg.states.total_wrench.force.y
    # z = msg.states.total_wrench.force.z


rospy.init_node('force_graph')

# fig=plt.figure()
# plt.axis([0,1000,0,1])

# n=0
# T = list()
# R = list()
# H = list()
# index = x.index('total_wrench')

while not rospy.is_shutdown():
	force_sub = rospy.Subscriber('/bumper_states', ContactsState, callback)
	#print(x)
	#print(type(x))
	#if 'total_wrench' not in x :
	#	print("Yes, 'time' NOT found in List : ")
	#print"_________________________l_______"
	# print str(x)+" "+str(y)+" "+str(z)
	# H.append(height);
	# R.append(right);
	# T.append(n);
	# plt.cla()
	# plt.plot(T,R,label="righti_Distance");
	# plt.grid(True)
	# plt.xlim(left = max(0,n-125))
	# plt.xlim(right = n+50)
	# plt.pause(0.01)
	# n= n+1
	# print(n)
