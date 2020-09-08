#!/usr/bin/env python

import tf2_ros
import rospy
from geometry_msgs.msg import Twist, TransformStamped
from fiducial_msgs.msg import FiducialTransformArray


class MarkerFrameBroadcast:

    def __init__(self, broadcast_handle):
        self._broadcast_handle = broadcast_handle
        self._tag_location_sub = rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, queue_size=5,
                                                  callback=self.broadcast)
        self.found = False

    def broadcast(self, message):
        if message.transforms:
            trans = TransformStamped()
            trans.transform = message.transforms[0].transform
            trans.header.frame_id = 'bottom_optical_frame'
            trans.child_frame_id = 'marker_frame'
            trans.header.stamp = rospy.Time.now()
            print(trans)
            self._broadcast_handle.sendTransform(trans)


if __name__ == '__main__':

    rospy.init_node('marker_frame_broadcaster')
    frame_broadcast = MarkerFrameBroadcast(broadcast_handle=tf2_ros.TransformBroadcaster())
    rospy.spin()
