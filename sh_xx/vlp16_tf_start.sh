#! /bin/bash
sleep 1
source /home/nvidia/chuan/chuan_nav_ws/devel/setup.bash
#roslaunch velodyne_pointcloud VLP16_points.launch 

roslaunch chuan_bringup vlp16_tf_start.launch
