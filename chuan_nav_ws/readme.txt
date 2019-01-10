安装依赖：
sudo apt-get install ros-kinetic-map-server
sudo apt-get install ros-kinetic-navigation
sudo apt-get install ros-kinetic-turtlebot-msgs
sudo apt-get install ros-kinetic-joy

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
cmd_vel_mux/input/navi         5
速度输出：
mobile_base/commands/velocity
*********************************************88
