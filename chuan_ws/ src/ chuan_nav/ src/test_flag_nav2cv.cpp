#include "ros/ros.h"
//xx
#include <xx_msgs/Flag.h>

#include <iostream>
using namespace std;
ros::Publisher to_flag_pub;
char ch;
int main(int argc,char** argv)
{
	ros::init(argc, argv, "test_flag_pub");
    ros::NodeHandle n;	
    to_flag_pub = n.advertise<xx_msgs::Flag>("flag_nav_to_cv",1);
    while(ros::ok())
    {
        printf("please input:1:nav or 2:cv\n");
        ch = getchar();   //按键模拟控制权
        if(ch == '1')    
        {
            xx_msgs::Flag flag_cv;
            flag_cv.flag = "nav start,cv stop";
            to_flag_pub.publish(flag_cv);
            cout<<flag_cv.flag<<endl;
        }
        if(ch == '2')
        {
            xx_msgs::Flag flag_cv;
            flag_cv.flag = "nav stop,cv start";
            to_flag_pub.publish(flag_cv);
            cout<<flag_cv.flag<<endl;
        }
    }	
    return 0;
}
