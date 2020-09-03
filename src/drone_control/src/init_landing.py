#!/usr/bin/env python

import rospy
from drone_control_detection.drone_camera_detection import DroneCameraDetection
if __name__ == '__main__':

    rospy.init_node('initiate_landing')
    drone_camera_detection = DroneCameraDetection()
    rospy.spin()

