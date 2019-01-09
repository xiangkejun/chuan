//xx
// 用于系统内标志位传递使用
// 服务器只能存在一个

#include "ros/ros.h"
#include "xx_msgs/Flag_xx.h"
#include <iostream>
bool flag(xx_msgs::Flag_xx::Request  &req,
         xx_msgs::Flag_xx::Response &res)
{
    std::string flag_req;
    std::string flag_A = "ok";
    flag_req = req.flag;
    if(flag_req == flag_A)   // "ok" 
    {
        res.result= "I am ok";
    }
    else
    {
        res.result="you input error!!";
    }
    std::cout<<"request: flag="<<req.flag<<std::endl;
    std::cout<<"sending back response: "<<res.result<<std::endl;
    //ROS_INFO("request: flag=%s,",  req.flag);
    //ROS_INFO("sending back response: [%s]", res.result);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "flag_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("flag_xx", flag);
    ROS_INFO("Ready to recieve a flag...");
    ros::spin();

    return 0;
}