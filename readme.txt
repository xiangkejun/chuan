//xx
//使用VLP16 进行导航
//3dlidar box 跟随测试成功
//标志位跟新表示（nav ->3dlidar -> cv）
//添加控制权优先级
//加入范围点到达后调整角度方便图像进一步的处理（nav_control.cpp）

***************************************
// xx 20190108
//nav->3dlidar->cv控制权模拟测试成功
启动:
在tk1上 ./nav_norviz.sh  (会启动vlp16)
在control上: ./nav_control.sh  ./box_pub.sh  ./box_gensui.sh  
./test_pub.sh

快捷启动方式1：
在TK1上：./nav_norviz.sh
control上：  ./nav_xx.sh   
./test_pub.sh 开启第一个点

快捷启动方式2： 
20190108   xx
在TK1上：./nav_norviz.sh   ./control_norviz.sh(定点+box_gensui)
control上：./test_pub 第一个点
***************************************
chuan_nav_ws:
1.实现定点导航，导航程序运行在tk1上，
2. ./nav_rviz.sh 可以在control机上查看rviz图。


chuan_ws:
1.chuan_control :底层驱动控制
    三维box 跟踪测试

2. chuan_nav:
1)导航权交接控制（与图像 ）
2)模拟标志位进行进行定点导航
nav_control.cpp //添加通过距离和角度在一定范围到达目标点的方法
nav_control3.cpp //通过statusCallback()确定目标点的到达

velodyne_ws:
1.驱动激光
<<<<<<< HEAD
=======

××××××××××××××××××××××××××××××××××××
船底盘启动
./keyboard_chuan.sh
./turtlebotkey_chuan.sh

***********************
激光box跟随测试：
在tk1上启动 ./turtlebot_qidong.sh
在control上的chuan_ws启动   ./test_3dlidar_pub.sh   ./box_gensui.sh  ./bag_xx.sh或./vlp16_start.sh
************************************************************
导航定点与图像测试：
目前共有3个点
double nav_point[3][4]={
//0,0,0,1,     //yuan dian
2.669,-0.169,-0.047,0.999,
3.385,2.632,0.104,0.994,
};
在tk1上 启动 ./nav_norviz.sh
在control 的chuan_nav_ws上可以启动 ./nav_rviz.sh
在chuan_ws   ./nav_control.sh     ./test_pub.sh发布地一个点
*********************************************************

