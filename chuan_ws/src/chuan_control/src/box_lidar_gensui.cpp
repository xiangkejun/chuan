//
// box_lidar_points 
//获取激光点云的box 并跟踪最近的物体 
//添加控制权交接
//能控制小车运动
// BOX处理完后停止box cv start
//找到半径7m前半圆的 box ，并跟踪 1<r<7内的box 当r<1,stop

// darknet_ros_msgs
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <xx_msgs/Flag.h>

#include <string>
#include <iostream>
#include <stdio.h>

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

// Header header
// geometry_msgs/Pose pose
// geometry_msgs/Vector3 dimensions  # size of bounding box (x, y, z)
// # You can use this field to hold value such as likelihood
// float32 value
// uint32 label


ros::Publisher vel_pub;  // 发布速度
ros::Publisher lidartocv_flag_pub;
ros::Publisher navto3dlidar_flag_pub;

float distance_xy_min,distance_xy_tmp,distance_now;
int j;
const float distance_goal=1;
const float line_scale_x = 0.1;
const float y_scale = 0.1;
void box_lidar_yoloCallback(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& box_msg)
{
    //接收到的点云是在半径60m 的前半圆体内
    // std::cout<< "123";
     std::cout<< box_msg->header;
     std::cout<<"num of box_lidar_points :"<<box_msg->boxes.size()<<std::endl;
     if( box_msg->boxes.size() == 0)   //no  box
     {
         std::cout << "no box" << std::endl;
         vel_pub.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));   //stop
     }
    if( box_msg->boxes.size() > 0)
    { // 找最近的 box   7< box_distance<20
        j=0;
        distance_xy_min = sqrt(pow(box_msg->boxes[0].pose.position.x,2) + pow(box_msg->boxes[0].pose.position.y,2));
        for(int i=0; i<box_msg->boxes.size();i++)
        {   
            // std::cout<<i<<" box center x = "<< box_msg->boxes[i].pose.position.x <<std::endl;
            // std::cout<<i<<" box center y = "<< box_msg->boxes[i].pose.position.y <<std::endl;
            // std::cout<<i<<" box center z = "<< box_msg->boxes[i].pose.position.z <<std::endl;
            // std::cout<<i<<" box length x = "<< box_msg->boxes[i].dimensions.x <<std::endl;
            // std::cout<<i<<" box width y = "<< box_msg->boxes[i].dimensions.y <<std::endl;
            // std::cout<<i<<" box height z = "<< box_msg->boxes[i].dimensions.z <<std::endl;
            // std::cout<<"-------------------------------------"<<std::endl;
            // std::cout<<"box label = "<< box_msg->boxes[i].label;
            // std::cout<<"box value = "<< box_msg->boxes[i].value;


            distance_xy_tmp = sqrt(pow(box_msg->boxes[i].pose.position.x,2) + pow(box_msg->boxes[i].pose.position.y,2));
            if(distance_xy_tmp >7)
            {
                continue; // <5  or > 50m 不做处理
            }
            if(distance_xy_tmp < distance_xy_min)
            {
                distance_xy_min = distance_xy_tmp;
                j = i;  //最近box 的那个编号
            }
        }
        std::cout << "zuijin box number j = " <<j << std::endl;
        // zuijin box 一定距离跟踪
        distance_now = sqrt(pow(box_msg->boxes[j].pose.position.x,2) + pow(box_msg->boxes[j].pose.position.y,2));
        if(distance_now >  distance_goal)   //7m
        {       
            geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
            cmd->linear.x = (distance_now - distance_goal) * line_scale_x;
            // 朝左了 向右偏， 朝右了，向左偏
            cmd->angular.z = box_msg->boxes[j].pose.position.y * y_scale;

            if(cmd->linear.x > 0.2) cmd->linear.x = 0.2;
            if(cmd->angular.z > 0.5) cmd->angular.z = 0.5;
            vel_pub.publish(cmd);
            std::cout<<"box_vx= "<<cmd->linear.x<<"  box_vz= "<<cmd->angular.z<<std::endl;

        }
        else    // box在 < 7m 的位置时 停止
        {   
            xx_msgs::Flag flag_nav_to_3dlidar;
            flag_nav_to_3dlidar.flag = "3dbox need stop";
            navto3dlidar_flag_pub.publish(flag_nav_to_3dlidar);  //停止box
            std::cout<<flag_nav_to_3dlidar.flag<<std::endl;

            xx_msgs::Flag flag_3dlidar_to_cv;
            flag_3dlidar_to_cv.flag = "nav stop,cv start";
            lidartocv_flag_pub.publish(flag_3dlidar_to_cv);   //发布图像控制标志
            std::cout<<flag_3dlidar_to_cv.flag<<std::endl;

            vel_pub.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));   //stop
        }
    }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "box_lidar_points");
  ros::NodeHandle n,n1;
  //当 /box_lidar_points 没有发布时 不会进行box 的寻找
  ros::Subscriber sub = n.subscribe("/box_lidar_points", 1, box_lidar_yoloCallback);
  //ros::Rate loop_rate(10);
  //vel_pub = n.advertise<geometry_msgs::Twist>("chuan_vel",10);
 // vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);
  vel_pub = n1.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",100);

  lidartocv_flag_pub = n.advertise<xx_msgs::Flag>("flag_nav_to_cv",1);    //发布控制交接权到图像标记
  navto3dlidar_flag_pub = n.advertise<xx_msgs::Flag>("flag_nav_to_3dlidar",1); //发布3d激光停止用(2个地方有发布)

  ros::spin();
  return 0;
}
