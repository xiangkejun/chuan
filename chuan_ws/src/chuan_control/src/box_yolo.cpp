//
// box_yolo 
//获取图像中的box

// darknet_ros_msgs
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

#include <string>
#include <iostream>
#include <stdio.h>

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
//  std::string  Class_;
//  float probability_;
//  int xmin_;
//  int ymin_;
//  int xmax_;
//  int ymax_;

//  geometry_msgs::Twist vel_msg;
// float vx,vz;

 ros::Publisher vel_pub;

void box_yoloCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& box_msg)
{
  //  probability_ = box_msg->bounding_boxes.probability;
    std::cout<< "123";
    std::cout<< box_msg->header;
    std::cout<<"num of box_images:" <<box_msg->bounding_boxes.size() <<std::endl;
    for(int i=0; i<box_msg->bounding_boxes.size();i++)
    {

        std::cout<<"class= "<<box_msg->bounding_boxes[i].Class<<std::endl;
        std::cout<<"probability= "<<box_msg->bounding_boxes[i].probability<<std::endl;
        std::cout<<"xmin= "<<box_msg->bounding_boxes[i].xmin<<std::endl;
        std::cout<<"ymin= "<<box_msg->bounding_boxes[i].ymin<<std::endl;
        std::cout<<"xmax= "<<box_msg->bounding_boxes[i].xmax<<std::endl;
        std::cout<<"ymax= "<<box_msg->bounding_boxes[i].ymax<<std::endl;
        std::cout<<"-----------------------------------"<<std::endl;
    }
    
    
    // vel_msg.linear.x = vx;
    // vel_msg.angular.z =vz;
    // vel_pub.publish(vel_msg);
  
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "box_yolo");
  ros::NodeHandle n,n1;
  ros::Subscriber sub = n.subscribe("/darknet_ros/bounding_boxes", 1, box_yoloCallback);

  //ros::Rate loop_rate(10);
 // vel_pub = n1.advertise<geometry_msgs::Twist>("chuan_vel",10);

  ros::spin();
  return 0;
}
