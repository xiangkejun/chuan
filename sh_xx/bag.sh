#! /bin/bash
sleep 1
#rosbag play bag/g1.bag -r 1
rosbag record -O ~/bag/1  /tf  /scan  

##rosbag record -O bag/g1 /tf  /scan   
