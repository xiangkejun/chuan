#include "ros/ros.h"

//xx
#include <xx_msgs/Flag.h>

#include <iostream>
using namespace std;

ros::Publisher to_flag_pub;


char ch;
int main(int argc,char** argv)
{
	ros::init(argc, argv, "flag_tuolian_pub");
    ros::NodeHandle n;	

	
    to_flag_pub = n.advertise<xx_msgs::Flag>("flag_tuolian_start",1);
    while(ros::ok())
    {
        printf("please input:1:tuolian start\n");
        ch = getchar();   //按键模拟控制权
        if(ch == '1')    
        {
            xx_msgs::Flag flag_cv;
            flag_cv.flag = "tuolian start";
            to_flag_pub.publish(flag_cv);
            cout<<flag_cv.flag<<endl;
        }
    }
//	ros::spin();	
    return 0;
}