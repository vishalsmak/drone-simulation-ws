#!/usr/bin/env python

from drone_control_detection.aruco_pose_detection import *

if __name__ == "__main__":
    # --- Define Tag
    id_to_find = 50
    marker_size = 10  # - [cm]

    rospy.init_node('detect_aruco_tag_pose')

    # --- Get the camera calibration path
    calib_path = "/home/vishal/vishal_testing/drone-simulation-ws/src/drone_control/config/"
    camera_matrix = np.loadtxt(calib_path + 'cameraMatrix.txt', delimiter=',')
    camera_distortion = np.loadtxt(calib_path + 'cameraDistortion.txt', delimiter=',')
    aruco_tracker = ArucoPoseDetection(id_to_find=50, marker_size=10, show_video=True, camera_matrix=camera_matrix,
                                       camera_distortion=camera_distortion)

    rospy.spin()

