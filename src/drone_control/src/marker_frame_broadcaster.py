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
        # Run this loop at about 10Hz
        if message.isTagVisible:
            rospy.sleep(0.1)
            self.x += 1
            if self.x > 3:
                self.x = 0
            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = "world"
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = "carrot1"
            t.transform.translation.x = 0.0
            t.transform.translation.y = self.x
            t.transform.translation.z = 0.0

            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            tfm = tf2_msgs.msg.TFMessage([t])
            self.pub_tf.publish(tfm)

if __name__ == '__main__':

    rospy.init_node('fixed_tf2_broadcaster')
    tfb = FixedTFBroadcaster()

    rospy.spin()