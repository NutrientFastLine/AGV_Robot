#!/bin/bash

source /opt/ros/melodic/setup.bash
source /home/nanorobot/catkin_ws/devel/setup.bash
sleep 2
roslaunch robot_package robot_map.launch &
echo "start map!!!"
wait
exit 0
