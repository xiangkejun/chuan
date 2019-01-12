//xx
//测试 定点导航
// 测试成功
//测试与图像通信
//添加 3dlidar
//加入在一定距离范围内定点，并将角度调整到需要的方向 并交接控制权
#include "ros/ros.h"
//#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
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
ros::Publisher vel_pub;	// 角度调整
ros::Subscriber sub_status;  //订阅当前导航状态
ros::Subscriber odom_sub;

ros::Publisher cancle_pub;
ros::Publisher lidartocv_flag_pub;
ros::Publisher navto3dlidar_flag_pub;
ros::Subscriber recv_cv_flag_sub;
//ros::Subscriber recv_3dlidar_flag_sub;


bool frist = true;
int flag_cv2nav_status = 0;   //导航标志
int flag_3dlidar_to_cv_status;  //3d及
int flag_nav_status = 0;
int flag_degree_do = 0;
int current_point=0;  //当前导航点
int current_status=0,last_current_status=0;
double currect_x=0,currect_y=0,current_orientation_z=0;
double current_orientation_w=0;
double nav_point[3][4]={
//0,0,0,1,     //yuan dian
// hokuyo 室内的两个点
3.023,-0.458,0.634,0.773,
3.164,5.525,0.720,0.694,

// vlp16 导航点 lab out
// 2.168,0.423,0.035,0.999,
// 11.377,1.681,0.114,0.993,
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


// void do_3d_lidar()
// {
// 	actionlib_msgs::GoalID empty_goal;
//     cancle_pub.publish(empty_goal);    //取消导航

//     xx_msgs::Flag flag_nav_to_3dlidar;
//     flag_nav_to_3dlidar.flag = "nav stop,3dlidar start";
//     navto3dlidar_flag_pub.publish(flag_nav_to_3dlidar);    //导航停止，启动3D激光扫描垃圾
  
// }
//改到在box 处理完后发布
void do_image()
{
    // actionlib_msgs::GoalID empty_goal;
	// cancle_pub.publish(empty_goal); //取消导航

	xx_msgs::Flag flag_3dlidar_to_cv;
	flag_3dlidar_to_cv.flag = "nav stop,cv start";
	lidartocv_flag_pub.publish(flag_3dlidar_to_cv);   //发布图像控制标志
}

std::string flag_cv;
void recv_cv_flag_callback(const xx_msgs::Flag::ConstPtr& msg)  //接收与图像控制权标记
{
	flag_cv = msg->flag;
	std::cout<<flag_cv<<std::endl;
	if(flag_cv == "nav start,cv stop")
	{
		flag_cv2nav_status = 1; //用于设置目标点
		flag_nav_status = 1;  // 正处于导航状态
		cout<<"nav do start"<<endl;
	}
}


void *pub_point(void *arg)  //发布dian
{
    while(ros::ok())
    {  
      if(flag_cv2nav_status == 1 )
	  {
		flag_cv2nav_status = 0;
        sleep(1);
	//	current_point++;
		if(current_point == 2)   //回到起点
		{
			current_point = 0;  //原点
			pub_switch();   
		}
		cout <<"current_point" << current_point<<endl;
		pub_switch();  //设置下一个目标点
		current_point++;
	  }
    }
}

void *degree_complete(void *arg)
{
	while(ros::ok())
	{
		if(flag_degree_do == 1)	//开始调整角度
		{
            geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
            cmd->linear.x = 0;
            cmd->angular.z = 0.1;
            vel_pub.publish(cmd);    

			if(fabs(current_orientation_z - 0.0)<0.3)
			{
				flag_degree_do = 0; //角度到位

				xx_msgs::Flag flag_3dlidar_to_cv;
				flag_3dlidar_to_cv.flag = "nav stop,cv start";
				lidartocv_flag_pub.publish(flag_3dlidar_to_cv);   //发布图像控制标志

				flag_nav_status = 0; //状态转移到cv
			}
		}
	}
}


void create_all_thread(void)
{

	pthread_t pub_thread ;
	pthread_t degree_thread;
	if( (pthread_create( &pub_thread , NULL , pub_point , NULL )) != 0 )
	{
		perror("Create the thread fail");
		exit( 1 );
	}
	if(pthread_create(&degree_thread, NULL,degree_complete,  NULL)!=0)
	{  
	perror("pthread_create error!");  
	exit( 1 );
	} 
}

void statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{
  //  cout <<"goal_n"<<goal_n<<endl;
	 if (goal_n>0)
	{	
	 	last_current_status=current_status;
	 	current_status=msg->status_list[0].status;
    //     cout<<"current_status" << current_status<<endl;
	 	if ((current_status==3)&&(last_current_status!=current_status))  // 成功到达目的地  SUCCEEDED
    //    if (current_status==3)
		{
				
			cout<<current_point<<" navigation reach!!"<<endl;
			sleep(1);
			// switch(current_point)
			// {
			// 	case 0: 
            //     //    do_3d_lidar();  //3dlidar start
            //         do_image(); //到达第一个点
			// 		cout<<"000000000"<<endl;
            //         break;
			// 	case 1: 
            //     //    do_3d_lidar();
            //         do_image(); //到达第二个点
			// 		cout<< "11111111111111"<<endl;
            //         break;
            //     case 2: 
            //     //    do_3d_lidar();
            //         do_image(); //处理完后回到原点
			// 		cout<<"222222222"<<endl;
            //         break;
			// }
		}
		// else if((current_status==4)&&(last_current_status!=current_status))  //目标被流产 ABORTED 状态出现了错误的情况
		// {
		// 	current_point++;	
	 	// 	pub_switch();
	  	// }		
	  }
}


void odom_pose_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
	double diff_x,diff_y,diff_dian;
	currect_x = msg->pose.pose.position.x;
	currect_y = msg->pose.pose.position.y;
	current_orientation_z = msg->pose.pose.orientation.z;
//	current_orientation_w = msg->pose.pose.orientation.w;

	diff_x = currect_x - nav_point[current_point][0];
	diff_y = currect_y - nav_point[current_point][1];
	diff_dian = sqrt(diff_x*diff_x + diff_y*diff_y);
	cout<<"diff_x= "<<diff_x<<"diff_y= "<<diff_y<<"diff_dian= "<<diff_dian<<endl;
	if(flag_nav_status)  //导航状态
	{
		if(diff_dian <= 0.3)
		{
			flag_degree_do = 1;
			actionlib_msgs::GoalID empty_goal;
			cancle_pub.publish(empty_goal); //取消导航
		}
	}
}

int main(int argc,char** argv)
{
	ros::init(argc, argv, "chuan_nav");
    ros::NodeHandle n;	

	pub_goal = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",10);	//发布导航点
	vel_pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);	//向底盘发送速度
    cancle_pub = n.advertise<actionlib_msgs::GoalID>("move_base/cancel",10);  //用于取消导航
	sub_status = n.subscribe("move_base/status",10,statusCallback);	//订阅是否到达目标点
	odom_sub = n.subscribe("odom",10,odom_pose_callback);   //用于范围距离判断

    lidartocv_flag_pub = n.advertise<xx_msgs::Flag>("flag_nav_to_cv",1);    //发布控制交接权到图像标记
//	navto3dlidar_flag_pub = n.advertise<xx_msgs::Flag>("flag_nav_to_3dlidar",1); //发布控制权交给3d激光
	recv_cv_flag_sub = n.subscribe<xx_msgs::Flag>("flag_cv_to_nav",1,recv_cv_flag_callback);  //从图像接收控制权标记

	create_all_thread();
	ros::spin();	
    return 0;
}
