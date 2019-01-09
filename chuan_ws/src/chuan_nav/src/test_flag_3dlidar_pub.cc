#include "ros/ros.h"

//xx
#include <xx_msgs/Flag.h>

#include <iostream>
using namespace std;

ros::Publisher to_flag_pub;


char ch;
int main(int argc,char** argv)
{
	ros::init(argc, argv, "test_flag_vlp_pub");
    ros::NodeHandle n;	

	
    to_flag_pub = n.advertise<xx_msgs::Flag>("flag_nav_to_3dlidar",1);
    while(ros::ok())
    {
        printf("please input:1:nav or 2:3dlidar\n");
        ch = getchar();   //按键模拟控制权
        if(ch == '1')    
        {
            xx_msgs::Flag flag_3dlidar;
            flag_3dlidar.flag = "nav stop,3dlidar start";
            to_flag_pub.publish(flag_3dlidar);
            cout<<flag_3dlidar.flag<<endl;
        }
        if(ch == '2')
        {
            xx_msgs::Flag flag_cv;
            flag_cv.flag = "3dbox need stop";
            to_flag_pub.publish(flag_cv);
            cout<<flag_cv.flag<<endl;
        }

    }

   // create_all_thread();

//	ros::spin();	
    return 0;
}
