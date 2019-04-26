#!/bin/bash
source /home/xx/chuan/chuan_nav_ws/devel/setup.bash
sleep 5
#roslaunch turtlebot_navigation amcl_chuan.launch
roslaunch chuan_navigation nav_chuan_hokuyo.launch
#roslaunch chuan_navigation nav_chuan_vlp16.launch
