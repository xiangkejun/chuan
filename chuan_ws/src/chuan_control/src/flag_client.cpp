//xx
// 用于系统内标志位传递使用
// 可以有多个客服端

#include "ros/ros.h"
#include "xx_msgs/Flag_xx.h"


#include <cstdlib>
#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "flag_client");
  
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<xx_msgs::Flag_xx>("flag_xx");
    xx_msgs::Flag_xx srv;

    std::string ch;
     while(ros::ok())
    {
        std::cout << "please input a string"<< std::endl;
        std::cin >> ch;

        srv.request.flag = ch;
        if (client.call(srv))
        {
            std::cout<<"reslut: " << srv.response<<std::endl;
       // ROS_INFO("Sum: %s", srv.response.result);
        }
        else
        {
        ROS_ERROR("Failed to call service flag_xx");
        return 1;
        }

     }
     
    return 0;
}