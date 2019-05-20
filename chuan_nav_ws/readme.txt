安装依赖：
sudo apt-get install ros-kinetic-map-server
sudo apt-get install ros-kinetic-navigation
sudo apt-get install ros-kinetic-turtlebot-msgs
sudo apt-get install ros-kinetic-joy
sudo apt-get install ros-melodic-tf2-sensor-msgs

sudo apt-get install ros-kinetic-velodyne-msgs 
sudo apt-get install ros-kinetic-velodyne-driver

http://wiki.ros.org/velodyne
注：先编译有driver-common的包
**************************************************
http://www.ncnynl.com/archives/201611/1097.html

roslaunch turtlebot_bringup minimal.launch

roslaunch turtlebot_navigation amcl_hokuyo.launch map_file:=/home/xx/map/lab.yaml

roslaunch turtlebot_rviz_launchers view_navigation.launch

//3维激光导航中加入避障  在costmap_common_params.yaml中修改。
//
********************************************************
//在 turtlebot_bringup/pararm/mux.yaml添加图像控制优先级。
//键盘是最高优先级。

速度输入：                     优先级
cmd_vel_mux/input/teleop       8
cmd_vel_mux/input/cv_vel       7
cmd_vel_mux/input/xbox_joy     6
cmd_vel_mux/input/navi         5
速度输出：
mobile_base/commands/velocity
*********************************************88

// 将kobuki中的/odom 改成了odom1  改回了/odom 

添加不适用激光只使用odom
测试： 
1. roslaunch turtlebot_navigation nav_odom.launch 
2. roslaunch scan_speed odom_tf.launch

// 20190409
在navigation包下添加 rrt_plan/rrt_planner作为 base_global_planner 并测试成功

//20190423
在chuan_bringup添加vlp16_tf_start.launch
实现vlp16激光的驱动并发布tf关系。


