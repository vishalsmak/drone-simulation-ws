#!/usr/bin/env python

from tf import TransformBroadcaster
import tf2_ros
import rospy
from rospy import Time
from drone_control.msg import MarkerPosition
from geometry_msgs.msg import Twist, TransformStamped
from geometry_msgs.msg import Quaternion
from tf import transformations as t
from fiducial_msgs.msg import FiducialTransformArray


class MarkerFrameBroadcast:

    def __init__(self, broadcast_handle):
        self.b = broadcast_handle
        self.translation = (1.0, 1.0, 0.0)
        self.rotation = (0.0, 0.0, 0.0, 1.0)
        self._tag_location_sub = rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, queue_size=5,
                                                  callback=self.broadcast)
        self.found = False
        # self.static_trans = None

    def broadcast(self, message):
        if message.transforms:
            # transform = message.transforms[0].transform
            # self.translation = transform.translation
            # self.rotation = transform.rotation
            trans = TransformStamped()
            trans.transform = message.transforms[0].transform
            trans.header.frame_id = 'bottom_optical_frame'
            trans.child_frame_id = 'marker_frame'
            # self.translation = (message.position.position.x, message.position.position.y, message.position.position.z)
            # # print(self.translation)
            # x = Quaternion(message.position.orientation.x, message.position.orientation.y, message.position.orientation.z,
            #                  message.position.orientation.w)
            # self.rotation = (x.x, x.y, x.z, x.w)
            # self.rotation = (0,0,0,1)

            # transform = t.concatenate_matrices(t.translation_matrix(self.translation), t.quaternion_matrix(self.rotation))
            # print(transform)

            # my_tf.setRotation(my_tf.getRotation().normalize())
            # for each in self.translation:
            #     if each != 0:
            #         print('something')
            # self.b.sendTransform(self.translation, self.rotation, Time.now(), '/marker_frame_temp', '/bottom_optical_frame')
            # self.b.sendTransform(self.translation, self.rotation, Time.now(), '/bottom_optical_frame', '/marker_frame')
            # static_trans = trans
            trans.header.stamp = rospy.Time.now()
            print(trans)
            # Publish a static transform
            # br = tf2_ros.TransformBroadcaster()
            self.b.sendTransform(trans)
        # else:
        #     pass
            # self.b.sendTransform(self.translation, self.rotation, Time.now(), '/bottom_optical_frame', 'marker_frame')
        
        

    # def broadcast(self, message):
    #     if message.isTagVisible:
    #         self.translation = (message.position.position.x, message.position.position.y, message.position.position.z)
    #         # print(self.translation)
    #         x = Quaternion(message.position.orientation.x, message.position.orientation.y, message.position.orientation.z,
    #                          message.position.orientation.w)
    #         self.rotation = (x.x, x.y, x.z, x.w)
    #         self.rotation = (0,0,0,1)

    #         # transform = t.concatenate_matrices(t.translation_matrix(self.translation), t.quaternion_matrix(self.rotation))
    #         # print(transform)

    #         # my_tf.setRotation(my_tf.getRotation().normalize())
    #         # for each in self.translation:
    #         #     if each != 0:
    #         #         print('something')
    #         self.b.sendTransform(self.translation, self.rotation, Time.now(), '/marker_frame_temp', '/bottom_optical_frame')
    #         # self.b.sendTransform(self.translation, self.rotation, Time.now(), '/bottom_optical_frame', '/marker_frame')
    #     else:
    #         pass
    #         # self.b.sendTransform(self.translation, self.rotation, Time.now(), '/bottom_optical_frame', 'marker_frame')



if __name__ == '__main__':

    rospy.init_node('marker_frame_broadcaster')
    frame_broadcast = MarkerFrameBroadcast(broadcast_handle=tf2_ros.TransformBroadcaster())
    rospy.spin()
