#!/usr/bin/env python

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
from sensor_msgs.msg import Range

import rospy, cv2, cv_bridge
import numpy as np
import cv2.aruco as aruco
import numpy

# --- Define Tag
id_to_find = 50
marker_size = 10  # - [cm]
distance = 1.0

height_speed = 0.3  # How fast the drone go up
desired_height = 1.2
take_off = "true"


class DroneCameraDetection:

    def __init__(self):
        global take_off
        # --- Get the camera calibration path
        self.twist = Twist()

        self.workspace_path = '/home/vishal/vishal_testing'
        self.camera_matrix = np.loadtxt(
            self.workspace_path + '/drone-simulation-ws/src/drone_control/src/cameraMatrix.txt', delimiter=',')
        self.camera_distortion = np.loadtxt(
            self.workspace_path + '/drone-simulation-ws/src/drone_control/src/cameraDistortion.txt', delimiter=',')

        # --- 180 deg rotation matrix around the x axis
        self.r_flip = np.zeros((3, 3), dtype=np.float32)
        self.r_flip[0, 0] = 1.0
        self.r_flip[1, 1] = -1.0
        self.r_flip[2, 2] = -1.0

        # --- Define the aruco dictionary
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        self.parameters = aruco.DetectorParameters_create()

        # --- Capture the camera video (this may also be a video or a picture)
        self.capture = cv_bridge.CvBridge()

        # -- Font for the text in the image
        self.font = cv2.FONT_HERSHEY_PLAIN

        self.height = 0
        self.downward = 0
        self.upword = 0

        self.height_sub = rospy.Subscriber('/sonar_height', Range, callback=self.height_callback)
        self.camera_image_sub = rospy.Subscriber('/bottom/camera/image', Image, queue_size=5, callback=self.main)
        self.drone_status_pub = rospy.Publisher('land', String, queue_size=1)
        self.velocity_command_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def height_callback(self, message):
        self.height = message.range

    def main(self, message):
        global take_off
        # -- Read the camera frame
        frame = self.capture.imgmsg_to_cv2(message, desired_encoding='bgr8')

        # -- Convert in gray scale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # remember, OpenCV stores color images in Blue, Green, Red

        # -- Find all the aruco markers in the image
        corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=self.aruco_dict, parameters=self.parameters,
                                                     cameraMatrix=self.camera_matrix, distCoeff=self.camera_distortion)

        request = "True"

        if ids is None or take_off == "true":
            self.twist.linear.z = 0.5
            self.velocity_command_pub.publish(self.twist)

        if ids is not None and ids[0] == id_to_find:
            take_off = "false"
            self.drone_status_pub.publish(request)
            print " Landing Pad Found"
            ret = aruco.estimatePoseSingleMarkers(corners, marker_size, self.camera_matrix, self.camera_distortion)

            # -- Unpack the output, get only the first
            rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]

            # -- Draw the detected marker and put a reference frame over it

            aruco.drawDetectedMarkers(frame, corners)
            aruco.drawAxis(frame, self.camera_matrix, self.camera_distortion, rvec, tvec, 10)

            # marker = cap.imgmsg_to_cv2(msg,desired_encoding='bgr8')
            marks = self.capture.imgmsg_to_cv2(message, desired_encoding='bgr8')
            marker = cv2.rectangle(marks, (corners[0][0][1][0], corners[0][0][1][1]),
                                   (corners[0][0][3][0], corners[0][0][3][1]), (45, 255, 90), -1)

            hsv = cv2.cvtColor(marker, cv2.COLOR_BGR2HSV)
            lower_yellow = numpy.array([29, 86, 6])
            upper_yellow = numpy.array([64, 255, 255])
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

            # Calculating the the mask coordinates on marker
            h, w, d = marker.shape
            top_search = 3
            bottom_search = top_search + 300
            mask[0:top_search, 0:w] = 0
            mask[bottom_search:h, 0:w] = 0
            M = cv2.moments(mask)

            if M['m00'] > 0:
                dx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                cv2.circle(marker, (dx, cy), 1, (0, 0, 255), -1)

                # Calculating the error value that how much motors has to rotate

                err_x = dx - w / 2
                err_y = cy - h / 2
                self.twist.linear.x = -float(err_y) / 95
                self.twist.angular.z = -float(err_x) / 95
                self.twist.linear.z = -0.2
                if self.height <= 0.5:
                    self.downward = 1
            self.velocity_command_pub.publish(self.twist)

        if self.downward == 1:
            self.twist.angular.z = 0
            self.twist.linear.x = 0
            self.twist.linear.z = -0.5
            print "landing"
            self.velocity_command_pub.publish(self.twist)

        # --- Display the frame
        cv2.imshow('bottom_camera', frame)

        # --- use 'q' to quit
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            cv2.destroyAllWindows()
