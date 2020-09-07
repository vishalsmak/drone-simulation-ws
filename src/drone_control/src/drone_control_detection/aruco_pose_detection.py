#!/usr/bin/env python

import numpy as np
import cv2, cv_bridge
import cv2.aruco as aruco
import sys, time, math
import rospy
from sensor_msgs.msg import Image
from drone_control.msg import MarkerPosition
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range


def _rotation_matrix_to_euler_angles(R):
    # Calculates rotation matrix to euler angles
    # The result is the same as MATLAB except the order
    # of the euler angles ( x and z are swapped ).

    def is_rotation_matrix(R):
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype=R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

    assert (is_rotation_matrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


class ArucoPoseDetection:
    def __init__(self,
                 id_to_find,
                 marker_size,
                 camera_matrix,
                 camera_distortion,
                 show_video
                 ):

        self.id_to_find = id_to_find
        self.marker_size = marker_size
        self._show_video = show_video

        self._camera_matrix = camera_matrix
        self._camera_distortion = camera_distortion

        self.is_detected = False
        self._kill = False

        # --- 180 deg rotation matrix around the x axis
        self._R_flip = np.zeros((3, 3), dtype=np.float32)
        self._R_flip[0, 0] = 1.0
        self._R_flip[1, 1] = -1.0
        self._R_flip[2, 2] = -1.0

        # --- Define the aruco dictionary
        self._aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        self._parameters = aruco.DetectorParameters_create()

        # --- Capture the camera video (this may also be a video or a picture)
        self._capture = cv_bridge.CvBridge()

        # -- Font for the text in the image
        self._font = cv2.FONT_HERSHEY_PLAIN

        self._t_read = time.time()
        self._t_detect = self._t_read
        self.fps_read = 0.0
        self.fps_detect = 0.0

        # -- funtion parameters
        self._verbose = True
        self._publish = True

        self._height = 0

        self._quaternion = [0] * 4
        self._q = [0] * 4

        self._camera_image_sub = rospy.Subscriber('/bottom/camera/image', Image, queue_size=5, callback=self.track)
        self._sonar_height_sub = rospy.Subscriber('/sonar_height', Range, queue_size=5, callback=self.height_callback)
        self._tag_location_pub = rospy.Publisher('/location/tag', MarkerPosition, queue_size=1)
        self._camera_location_pub = rospy.Publisher('/location/camera', MarkerPosition, queue_size=1)
        self._command_velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def _update_fps_read(self):
        t = time.time()
        self.fps_read = 1.0 / (t - self._t_read)
        self._t_read = t

    def _update_fps_detect(self):
        t = time.time()
        self.fps_detect = 1.0 / (t - self._t_detect)
        self._t_detect = t

    def height_callback(self, message):
        self._height = message.range

    def stop(self):
        self._kill = True

    def temporary_land(self, image, corners):

        # -- point detection logic (may not be used)
        marker = cv2.rectangle(image, (corners[0][0][1][0], corners[0][0][1][1]),
                               (corners[0][0][3][0], corners[0][0][3][1]), (45, 255, 90), -1)

        hsv = cv2.cvtColor(marker, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([29, 86, 6])
        upper_yellow = np.array([64, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Calculating the the mask coordinates on marker
        h, w, d = marker.shape
        top_search = 3
        bottom_search = top_search + 300
        mask[0:top_search, 0:w] = 0
        mask[bottom_search:h, 0:w] = 0
        M = cv2.moments(mask)
        twist = Twist()

        if M['m00'] > 0:
            dx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(marker, (dx, cy), 1, (0, 0, 255), -1)

            # Calculating the error value that how much motors has to rotate

            err_x = dx - w / 2
            err_y = cy - h / 2
            twist.linear.x = -float(err_y) / 95
            twist.angular.z = -float(err_x) / 95
            twist.linear.z = -0.2
            if self._height <= 0.5:
                twist.angular.z = 0
                twist.linear.x = 0
                twist.linear.z = -0.5
                print "landing"
        self._command_velocity_pub.publish(twist)

    def track(self, message):

        self._kill = False

        marker_found = False
        x = y = z = 0
        x_cam = y_cam = z_cam = 0

        if not self._kill:

            # -- Read the camera frame
            frame = self._capture.imgmsg_to_cv2(message, desired_encoding='bgr8')

            self._update_fps_read()

            # -- Convert in gray scale
            gray = cv2.cvtColor(frame,
                                cv2.COLOR_BGR2GRAY)  # -- remember, OpenCV stores color images in Blue, Green, Red

            # -- Find all the aruco markers in the image
            corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=self._aruco_dict,
                                                         parameters=self._parameters,
                                                         cameraMatrix=self._camera_matrix,
                                                         distCoeff=self._camera_distortion)

            if not ids is None and self.id_to_find in ids[0]:
                marker_found = True
                self._update_fps_detect()
                # -- ret = [rvec, tvec, ?]
                # -- array of rotation and position of each marker in camera frame
                # -- rvec = [[rvec_1], [rvec_2], ...]    attitude of the marker respect to camera frame
                # -- tvec = [[tvec_1], [tvec_2], ...]    position of the marker in camera frame
                ret = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self._camera_matrix,
                                                      self._camera_distortion)

                # -- Unpack the output, get only the first
                rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]

                x = tvec[0]
                y = tvec[1]
                z = tvec[2]

                # -- land to check frames
                self.temporary_land(frame, corners)

                # -- Draw the detected marker and put a reference frame over it
                aruco.drawDetectedMarkers(frame, corners)
                aruco.drawAxis(frame, self._camera_matrix, self._camera_distortion, rvec, tvec, 10)

                # -- Obtain the rotation matrix tag->camera
                R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
                R_tc = R_ct.T

                # -- Get the attitude in terms of euler 321 (Needs to be flipped first)
                roll_marker, pitch_marker, yaw_marker = _rotation_matrix_to_euler_angles(self._R_flip * R_tc)
                self._q = quaternion_from_euler(roll_marker, pitch_marker, yaw_marker)

                # -- Now get Position and attitude f the camera respect to the marker
                pos_camera = -R_tc * np.matrix(tvec).T
                x_cam = pos_camera[0]
                y_cam = pos_camera[1]
                z_cam = pos_camera[2]

                # print "Camera X = %.1f  Y = %.1f  Z = %.1f  - fps = %.0f"%(pos_camera[0], pos_camera[1], pos_camera[2],fps_detect)
                if self._verbose: print "Marker X = %.1f  Y = %.1f  Z = %.1f  - fps = %.0f" % (
                    tvec[0], tvec[1], tvec[2], self.fps_detect)

                if self._show_video:
                    # -- Print the tag position in camera frame
                    str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f" % (tvec[0], tvec[1], tvec[2])
                    cv2.putText(frame, str_position, (0, 100), self._font, 1, (0, 255, 0), 2, cv2.LINE_AA)

                    # -- Print the marker's attitude respect to camera frame
                    str_attitude = "MARKER Attitude r=%4.0f  p=%4.0f  y=%4.0f" % (
                        math.degrees(roll_marker), math.degrees(pitch_marker),
                        math.degrees(yaw_marker))
                    cv2.putText(frame, str_attitude, (0, 150), self._font, 1, (0, 255, 0), 2, cv2.LINE_AA)

                    str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f" % (
                        x_cam, y_cam, z_cam)
                    cv2.putText(frame, str_position, (0, 200), self._font, 1, (0, 255, 0), 2, cv2.LINE_AA)

                    # -- Get the attitude of the camera respect to the frame
                    roll_camera, pitch_camera, yaw_camera = _rotation_matrix_to_euler_angles(self._R_flip * R_tc)
                    str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f" % (
                        math.degrees(roll_camera), math.degrees(pitch_camera),
                        math.degrees(yaw_camera))
                    cv2.putText(frame, str_attitude, (0, 250), self._font, 1, (0, 255, 0), 2, cv2.LINE_AA)



            else:
                if self._verbose: print "Nothing detected - fps = %.0f" % self.fps_read

            if self._show_video:
                # --- Display the frame
                cv2.imshow('frame', frame)

                # --- use 'q' to quit
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    cv2.destroyAllWindows()

            if self._publish:
                msg_tag_location = MarkerPosition()
                msg_tag_location.isTagVisible = marker_found
                msg_tag_location.position.position.x = y / 100
                msg_tag_location.position.position.y = z / 100
                msg_tag_location.position.position.z = x / 100
                msg_tag_location.position.orientation.x = 0
                msg_tag_location.position.orientation.y = 0
                msg_tag_location.position.orientation.z = 0
                msg_tag_location.position.orientation.w = 1
                self._tag_location_pub.publish(msg_tag_location)

                msg_camera_location = MarkerPosition()
                msg_camera_location.isTagVisible = marker_found
                msg_camera_location.position.position.x = x_cam
                msg_camera_location.position.position.y = y_cam
                msg_camera_location.position.position.z = z_cam
                msg_camera_location.position.orientation.x = self._quaternion[0]
                msg_camera_location.position.orientation.y = self._quaternion[1]
                msg_camera_location.position.orientation.z = self._quaternion[2]
                msg_camera_location.position.orientation.w = self._quaternion[3]
                self._camera_location_pub.publish(msg_camera_location)

                return marker_found, x, y, z
