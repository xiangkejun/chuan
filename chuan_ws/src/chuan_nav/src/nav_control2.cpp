//xx
//测试 定点导航
// 测试成功

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include "actionlib_msgs/GoalID.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "actionlib_msgs/GoalStatus.h"

//xx
#include <xx_msgs/Flag.h>

#include <iostream>
using namespace std;
//发布订阅节点
ros::Publisher pub_goal;  //发布目标点
ros::Publisher pub_vel;	//向cmd_vel话题发送数据
ros::Subscriber sub_status;  //订阅当前导航状态
ros::Publisher cancle_pub;
ros::Publisher lidartocv_flag_pub;
ros::Publisher navto3dlidar_flag_pub;
ros::Subscriber recv_cv_flag_sub;
ros::Subscriber recv_3dlidar_flag_sub;

bool frist = true;
int flag_cv2nav_status = 0;   //导航标志
int flag_3dlidar_to_cv_status;  //3d及
int current_point=0;  //当前导航点
int current_status=0,last_current_status=0;
double nav_point[3][4]={
0,0,0,1,     //yuan dian
2.669,-0.169,-0.047,0.999,
3.385,2.632,0.104,0.994,
};

int goal_n=0;	
void set_goal(int i)
{
	ros::Rate loop_rate(5);
	geometry_msgs::PoseStamped msg;
	msg.header.frame_id = "map";
	msg.header.stamp = ros::Time::now();
	msg.pose.position.x=nav_point[i][0];
	msg.pose.position.y=nav_point[i][1];
	msg.pose.orientation.z=nav_point[i][2];
	msg.pose.orientation.w=nav_point[i][3];
	pub_goal.publish(msg);
	cout<<"set goal"<<i<<endl;
    cout<<"goal point at "<<	nav_point[i][0]<<" "<<nav_point[i][1]<<" "
      <<nav_point[i][2]<< "  "<<nav_point[i][3]<<endl;
	loop_rate.sleep();
	goal_n++;
}

//最多4个点
void pub_switch()
{
	if(current_point>=0&&current_point<=3)	
	{
		set_goal(current_point);
	}
	else
	{
		fprintf(stderr,"Unsupported stop bits\n");
	}	
}
void send_box_lidar_vel()
{
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        pub_switch();
    }
}


void do_3d_lidar()
{
	actionlib_msgs::GoalID empty_goal;
    cancle_pub.publish(empty_goal);    //取消导航

    xx_msgs::Flag flag_nav_to_3dlidar;
    flag_nav_to_3dlidar.flag = "nav stop,3dlidar start";
    navto3dlidar_flag_pub.publish(flag_nav_to_3dlidar);    //导航停止，启动3D激光扫描垃圾
    while(ros::ok())
    {
		//一直等待直到3dbox处理完
        if(flag_3dlidar_to_cv_status == 1)  //等到3d激光扫描到垃圾且到达垃圾位置附近 退出
        {
			flag_3dlidar_to_cv_status = 0;
            return ;       //退出后，图像接着继续处理
        }
    } 
}

void do_image()
{
    actionlib_msgs::GoalID empty_goal;
	cancle_pub.publish(empty_goal); //取消导航

	xx_msgs::Flag flag_3dlidar_to_cv;
	flag_3dlidar_to_cv.flag = "nav stop,cv start";
	lidartocv_flag_pub.publish(flag_3dlidar_to_cv);   //发布图像控制标志
	while(ros::ok())
	{
		//一直等待 直到有指令返回
		if(flag_cv2nav_status == 1)   //有标志位返回时 ，开始去下一个点
		{
            flag_cv2nav_status = 0;
			 current_point++;	
			if(current_point == 2)   //回到起点
			{
				current_point = 0;  //原点
				pub_switch();   
			}
            cout <<"current_point" << current_point<<endl;
	 		pub_switch();  //设置下一个目标点
            sleep(1);
			break;   //退出整个循环
		}
	}
}


std::string flag_cv;
void recv_cv_flag_callback(const xx_msgs::Flag::ConstPtr& msg)  //接收与图像控制权标记
{
	flag_cv = msg->flag;
	std::cout<<flag_cv<<std::endl;
	if(flag_cv == "nav start,cv stop")
	{
		flag_cv2nav_status =1;
		cout<<"nav do start"<<endl;
        
	}
}
std::string flag_3dlidar;
void recv_3dlidar_flag_callback(const xx_msgs::Flag::ConstPtr& msg)
{
    flag_3dlidar = msg->flag;
    cout<< flag_3dlidar <<endl;
    if(flag_3dlidar == "3dlidar stop,cv start")
    {
        flag_3dlidar_to_cv_status = 1;
    }
}




void *pub_point(void *arg)  //发布dian
{
    while(ros::ok())
    {  
      if(flag_cv2nav_status == 1 )
	  {
		flag_cv2nav_status = 0;
		current_point++;
		if(current_point == 3)   //回到起点
		{
			current_point = 0;  //原点
			pub_switch();   
		}
		cout <<"current_point" << current_point<<endl;
		pub_switch();  //设置下一个目标点
		sleep(1);
	  }
    }
}



void create_all_thread(void)
{

	pthread_t pub_thread ;
		
	if( (pthread_create( &pub_thread , NULL , pub_point , NULL )) != 0 )
	{
		perror("Create the thread fail");
		exit( 1 );
	}
}

void statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{
  //  cout <<"goal_n"<<goal_n<<endl;
	 if (goal_n>0)
	{	
	// 	last_current_status=current_status;
	 	current_status=msg->status_list[0].status;
    //     cout<<"current_status" << current_status<<endl;
	// 	if ((current_status==3)&&(last_current_status!=current_status))  // 成功到达目的地  SUCCEEDED
        if (current_status==3)
		{
				
			cout<<current_point<<" navigation reach!!"<<endl;
			sleep(1);
			switch(current_point)
			{
				// case 0: 
                //   //  do_3d_lidar();
                //     do_image(); //到达第一个点
				// 	cout<<"000000000"<<endl;
				// case 1: 
                //  //   do_3d_lidar();
                //     do_image(); //到达第二个点
				// 	cout<< "11111111111111"<<endl;
                // case 2: 
                //    // do_3d_lidar();
                //     do_image(); //处理完后回到原点
					cout<<"222222222"<<endl;
			}
		}
		// else if((current_status==4)&&(last_current_status!=current_status))  //目标被流产 ABORTED 状态出现了错误的情况
		// {
		// 	current_point++;	
	 	// 	pub_switch();
	  	// }		
	  }
}

int main(int argc,char** argv)
{
	ros::init(argc, argv, "chuan_nav");
    ros::NodeHandle n;	

	pub_goal = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",10);	//发布导航点
	//pub_vel = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 100);	//向底盘发送速度
    cancle_pub = n.advertise<actionlib_msgs::GoalID>("move_base/cancel",10);  //用于取消导航
	sub_status = n.subscribe("move_base/status",10,statusCallback);	//订阅是否到达目标点

    lidartocv_flag_pub = n.advertise<xx_msgs::Flag>("flag_nav_to_cv",1);    //发布控制交接权到图像标记
	navto3dlidar_flag_pub = n.advertise<xx_msgs::Flag>("flag_nav_to_3dlidar",1); //发布控制权交给3d激光
	recv_cv_flag_sub = n.subscribe<xx_msgs::Flag>("flag_cv_to_nav",1,recv_cv_flag_callback);  //从图像接收控制权标记
	recv_3dlidar_flag_sub = n.subscribe<xx_msgs::Flag>("flag_3dlidar_to_cv",1,recv_3dlidar_flag_callback); //从3维激光接收标记

	create_all_thread();
	ros::spin();	
    return 0;
}