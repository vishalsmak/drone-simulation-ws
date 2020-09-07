#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
from drone_control.msg import MarkerPosition


class FixedTFBroadcaster:

    def __init__(self):
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        self.x = 0
        self._tag_detect_sub = rospy.Subscriber('/location/tag', MarkerPosition, queue_size=5, callback=self.display_frame)

    def display_frame(self, message):

        if message.isTagVisible:
            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = "front_cam_optical_frame"
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = "marker_frame"
            t.transform.translation.x = message.position.position.x
            t.transform.translation.y = message.position.position.y
            t.transform.translation.z = message.position.position.z

            t.transform.rotation.x = message.position.orientation.x
            t.transform.rotation.y = message.position.orientation.y
            t.transform.rotation.z = message.position.orientation.z
            t.transform.rotation.w = message.position.orientation.w

            tfm = tf2_msgs.msg.TFMessage([t])
            self.pub_tf.publish(tfm)

if __name__ == '__main__':

    rospy.init_node('fixed_tf2_broadcaster')
    tfb = FixedTFBroadcaster()

    rospy.spin()