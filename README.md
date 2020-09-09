sudo apt install ros-melodic-aruco-detect

roslaunch drone_control simulation.launch

rosservice call /enable_motors "enable: true"


