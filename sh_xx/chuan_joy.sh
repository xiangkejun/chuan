#!/bin/bash
sleep 1
export SUDO_ASKPASS=./_PWD_TEMP_
sudo -A  chmod 777 /dev/ttyUSB0
sleep 1
source /opt/ros/kinetic/setup.bash
source /home/xx/chuan_ws/devel/setup.bash
roslaunch  learning_joy  turtle_joy.launch
