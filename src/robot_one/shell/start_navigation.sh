#!/bin/bash

source /opt/ros/melodic/setup.bash
source /home/nanorobot/catkin_ws/devel/setup.bash
sleep 2
roslaunch robot_package start_navigation.launch &
echo "start map!!!"
wait

exit 0
