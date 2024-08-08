#!/bin/sh
#删除ros日至，避免长时间使用后，ros日志占据大量空间
gnome-terminal -t "rosclean" -x bash -c "rm -rf /home/gaolin/.ros/log"
# 睡眠1s
sleep 1s
#启动ROS节点管理器
gnome-terminal -t "rosmaster" -x bash -c "roscore"
sleep 1s

gnome-terminal -t "rosnode" -x bash -c "rosrun robot_one robot_one"
