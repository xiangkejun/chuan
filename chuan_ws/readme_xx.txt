
//xx

rosrun chuan_control box_lidar_points

roslaunch ros_cv  open_camera.launch 

roslaunch turtlebot_teleop keyboard_chuan.launch 


flag_xx 测试：

rosrun chuan_control flag_server
rosrun chuan_control flag_client
*******************************************************
激光box跟随测试：
在tk1上启动 ./turtlebot_qidong.sh
在control上的chuan_ws启动   ./test_3dlidar_pub.sh   ./box_gensui.sh  ./bag_xx.sh或 ./vlp16_start.sh
*******************************************************************
导航定点与图像测试：
目前共有3个点
double nav_point[3][4]={
0,0,0,1,     //yuan dian
2.669,-0.169,-0.047,0.999,
3.385,2.632,0.104,0.994,
};

在tk1上 启动 ./nav_norviz.sh
在control 的chuan_nav_ws上可以启动 ./nav_rviz.sh
在chuan_ws   ./nav_control.sh     ./test_pub.sh
**********************************************************
