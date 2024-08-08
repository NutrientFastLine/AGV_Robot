#!/bin/bash
source /opt/ros/melodic/setup.bash
source /home/nanorobot/catkin_ws/devel/setup.bash
sleep 0.5

roslaunch robot_package finished_build_map.launch 

sleep 5

killall roslaunch

echo "finished map!!!"


wait

exit 0
