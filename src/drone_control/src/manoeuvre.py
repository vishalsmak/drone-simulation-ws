#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped


class Manoeuvre:

    def __init__(self):
        self._height = 0
        self._height_sub = rospy.Subscriber('/pressure_height', PointStamped, self.height_callback)
        self._cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._twist = Twist()

    def height_callback(self, message):
        self._height = message.point.z

    def goto_height(self):
        if self._height:
            self._twist.angular.z = 0
            self._twist.linear.x = 0
            self._twist.linear.z = 0.5
        else:
            self._twist.angular.z = 0
            self._twist.linear.x = 0
            self._twist.linear.z = 0

        print "flying"
        self._cmd_vel_pub.publish(self._twist)


if __name__ == "__main__":
    rospy.init_node('fly_to_height')
    fly_drone = Manoeuvre()
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        fly_drone.goto_height()
        rate.sleep()
