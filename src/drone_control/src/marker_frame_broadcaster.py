#!/usr/bin/env python

from tf import TransformBroadcaster
import rospy
from rospy import Time
from drone_control.msg import MarkerPosition


class MarkerFrameBroadcast:

    def __init__(self, broadcast_handle):
        self.b = broadcast_handle
        self.translation = (1.0, 1.0, 0.0)
        self.rotation = (0.0, 0.0, 0.0, 1.0)
        self._tag_location_sub = rospy.Subscriber('/location/tag', MarkerPosition, queue_size=5,
                                                  callback=self.broadcast)

    def broadcast(self, message):
        if message.isTagVisible:
            message = MarkerPosition()
            self.translation = (message.position.position.x, message.position.position.y, message.position.position.z)
            self.rotation = (message.position.orientation.x, message.position.orientation.y, message.position.orientation.z,
                             message.position.orientation.w)
            self.b.sendTransform(self.translation, self.rotation, Time.now(), '/word', 'marker_frame')
        else:
            self.b.sendTransform(self.translation, self.rotation, Time.now(), '/world',
                                 'marker_frame')


def main():
    translation = (0.0, 0.0, 0.0)
    rotation = (0.0, 0.0, 0.0, 1.0)
    rate = rospy.Rate(5)  # 5hz

    x, y = 0.0, 0.0

    while not rospy.is_shutdown():
        if x >= 2:
            x, y = 0.0, 0.0

        x += 0.1
        y += 0.1

        translation = (x, y, 0.0)

        #b.sendTransform(translation, rotation, Time.now(), '/world', 'marker_frame')
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('marker_frame_broadcaster')
    frame_broadcast = MarkerFrameBroadcast(broadcast_handle=TransformBroadcaster())

    rospy.spin()