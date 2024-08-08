#!/bin/bash
source /opt/ros/melodic/setup.bash
source /home/nanorobot/catkin_ws/devel/setup.bash
rostopic pub /move_base/cancel actionlib_msgs/GoalID -- {} &
sleep 0.5
echo "cancel now nav goal!"
exit
