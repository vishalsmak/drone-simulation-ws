roslaunch drone_gazebo drone.launch

rosservice call /enable_motors "enable: true"

rosrun drone_control land.py
