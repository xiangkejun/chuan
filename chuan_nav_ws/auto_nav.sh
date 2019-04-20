#!/bin/bash
source /home/nvidia/chuan/chuan_nav_ws/devel/setup.bash
sleep 2
#roslaunch turtlebot_navigation amcl_chuan.launch

roslaunch chuan_navigation nav_chuan.launch
