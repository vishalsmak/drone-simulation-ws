#!/usr/bin/env python

import roslib

roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from drone_control.msg import TagLocation
import rospy

topic = 'visualization_marker_array'
cam_x = cam_y = cam_z = tag_x = tag_y = tag_z = 0


def point_copy_tag(message):
    global tag_x, tag_y, tag_z
    message = TagLocation()
    tag_x = message.cartesianLocation.x
    tag_y = message.cartesianLocation.y
    tag_z = message.cartesianLocation.z


def point_copy_cam(message):
    global cam_x, cam_y, cam_z
    message = TagLocation()
    cam_x = message.cartesianLocation.x
    cam_y = message.cartesianLocation.y
    cam_z = message.cartesianLocation.z


rospy.init_node('register')
publisher = rospy.Publisher(topic, Marker, queue_size=10)
_tag_location_sub = rospy.Subscriber('/location/tag', TagLocation, queue_size=1, callback=point_copy_tag)
_camera_location_sub = rospy.Subscriber('/location/camera', TagLocation, queue_size=1, callback=point_copy_cam)


count = 0

#global cam_x, cam_y, cam_z, tag_x, tag_y, tag_z
while not rospy.is_shutdown():
    marker = Marker()

    arrow_point_start = Point()
    arrow_point_end = Point()

    arrow_point_start.x = tag_x
    arrow_point_start.y = tag_y
    arrow_point_start.z = tag_z

    arrow_point_end.x = cam_x
    arrow_point_end.y = cam_y
    arrow_point_end.z = cam_z

    marker.header.frame_id = "/base_link"
    marker.type = marker.ARROW
    marker.action = marker.ADD
    marker.scale.x = 0.01
    marker.scale.y = 0.02
    marker.scale.z = 0.06
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.points.insert(0, arrow_point_start)
    marker.points.insert(1, arrow_point_end)
    marker.pose.orientation.w = 1.0

    publisher.publish(marker)

    rospy.sleep(0.01)
