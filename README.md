sudo apt install ros-melodic-aruco-detect

roslaunch drone_control transforms.launch

rosservice call /enable_motors "enable: true"

rosrun drone_control land.py
