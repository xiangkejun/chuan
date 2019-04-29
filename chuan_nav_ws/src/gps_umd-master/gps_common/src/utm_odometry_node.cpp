/*
* Translates sensor_msgs/NavSat{Fix,Status} into nav_msgs/Odometry using UTM
*/

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <stdio.h>
#include<cstdlib>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <tf/tf.h>
using namespace gps_common;
using namespace std;
static ros::Publisher odom_pub;
std::string frame_id, child_frame_id;
double rot_cov;
//float du;
double initeasting;
double initnorthing;
int control=0;
float x_1,y_1,z_1,w_1,xv_1,yv_1,zv_1,twistx,twisty;
//covert to r p y
std::string s;
void timeco_dt()
{
  char stime[256] = { 0 };
  time_t now_time;
  time(&now_time);
  strftime(stime, sizeof(stime), "%F %H:%M:%S", localtime(&now_time));
  s = stime;
  //std::cout << s << std::endl;
}

double getRoll(double x,double y,double z, double w)
{
double roll=atan2(2*(w*x+y*z),1-2*(x*x+y*y));
return roll;
}


double getPaw(double x,double y,double z, double w)
{
double yaw=asin(2*(w*y-x*z));
return yaw;
}

double getYitch(double x,double y,double z, double w)
{
double pitch=atan2(2*(w*z+x*y),1-2*(y*y+z*z));
return pitch;
}

/*
double getRoll(double x,double y,double z, double w)
{
double roll=atan2(2*(w*z+y*x),1-2*(x*x+z*z));
return roll;
}


double getPaw(double x,double y,double z, double w)
{
double yaw=asin(2*(w*x-y*z));
return yaw;
}

double getYitch(double x,double y,double z, double w)
{
double pitch=atan2(2*(w*y+x*z),1-2*(y*y+x*x));
return pitch;
}


double getRoll(double x,double y,double z, double w)
{
double roll=atan2(2*(w*x+y*z),1-2*(x*x+y*y));
return roll;
}


double getYaw(double x,double y,double z, double w)
{
double yaw=asin(2*(w*y-x*z));
return yaw;
}

double getPitch(double x,double y,double z, double w)
{
double pitch=atan2(2*(w*z+x*y),1-2*(y*y+z*z));
return pitch;
}

void turtlebotodom_callback(const nav_msgs::OdometryConstPtr& turtebotodom)
{

twistx=turtebotodom->twist.twist.linear.x;
twisty=turtebotodom->twist.twist.angular.z;

}
*/

/*
void imu_callback(const sensor_msgs::ImuConstPtr& imu)
{//ros::Rate loop_rate1(100);
x_1 = imu->orientation.x;
y_1 = imu->orientation.y;
z_1 = imu->orientation.z;
      w_1 =imu->orientation.w;
float yaw= getYitch( x_1,y_1,z_1, w_1);
du=yaw*180.0/3.1415;
printf("%fy d%f\n",yaw,du);	
twistx=sqrt(imu->linear_acceleration.x*imu->linear_acceleration.x+imu->linear_acceleration.z*imu->linear_acceleration.z);
twisty=imu->angular_velocity.y/180*3.1415926;
// printf("%fy d%f\n",yaw,du);	
// loop_rate1.sleep();
}
*/

void callback(const sensor_msgs::NavSatFixConstPtr& fix) 
{
  if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) 
  {
    ROS_INFO("No fix.");
    return;
  }
  x_1 =fix->position_covariance[0];
  y_1 =fix->position_covariance[1];
  z_1 =fix->position_covariance[2];
  w_1 =fix->position_covariance[3];   //get imu yaw

  double yaw,pitch,roll,du;
 // yaw= getYitch( x_1,y_1,z_1, w_1);
  tf::Matrix3x3 mat(tf::Quaternion(x_1,y_1,z_1, w_1));
  
  mat.getEulerYPR(yaw,pitch,roll);

  du=yaw*180.0/3.1415;
  printf("yaw= %0.10f du=%0.10f\n",yaw,du);	
  twistx=sqrt(fix->position_covariance[6]*fix->position_covariance[6]+fix->position_covariance[8]*fix->position_covariance[8]); //（（北速度平方+东速度平方）开方）
  twisty=fix->position_covariance[5]/180*3.1415926; //y轴角速度
  ros::Rate loop_rate(100);///////////////////////////////////xiugai  50---100
  if (fix->header.stamp == ros::Time(0)) 
  {
    return;
  }
  ofstream write;
  double northing, easting;
  std::string zone;
  // fix->latitude = 31.5367782;
  // fix->longitude=104.70144231;
  LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);     //gps 数据格式转换
 // LLtoUTM(31.53723986, 104.70158125, northing, easting, zone);     //gps 数据格式转换
 printf(" weidu=  %0.10lf jingdu=  %0.10lf\n",fix->latitude,fix->longitude);
 printf(" northing=  %0.10lf easting=  %0.10lf\n",northing,easting);
  if(control==0)    //对initeasting   initnorthing只赋一次初值
  {
    initeasting=442322.622197;   //转换后的初始值
    initnorthing= 3438871.211899;    //3438861.211899
    control=1;

   // northing=  3489131.54590202 easting=  471658.71730315
  }

  if (odom_pub) 
  {
    nav_msgs::Odometry odom;
    odom.header.stamp = fix->header.stamp;

    if (frame_id.empty())
      odom.header.frame_id = fix->header.frame_id;
    else
      odom.header.frame_id = frame_id;

    odom.child_frame_id = child_frame_id;
   // odom.pose.pose.position.x = easting-initeasting;
   // odom.pose.pose.position.y = northing-initnorthing;     //算相对原点的位置
    
    odom.pose.pose.position.x = 0;
    odom.pose.pose.position.y = 0;  //机器人处于地图原点
    
    odom.pose.pose.position.z = 0;//fix->altitude

    odom.pose.pose.orientation.x = x_1;
    odom.pose.pose.orientation.y = y_1;
    odom.pose.pose.orientation.z = z_1;
    odom.pose.pose.orientation.w = w_1;//  yaw的四元数形式 imu

    odom.twist.twist.linear.x=twistx;   //（（北速度平方+东速度平方）开方）

    // odom.twist.twist.linear.x = fix->position_covariance[8];//东
    // odom.twist.twist.linear.y = fix->position_covariance[6];//北

    odom.twist.twist.angular.z=twisty;  //y轴角速度  即正前方的角速度
    timeco_dt();   //获取本地时间 存在  s 中
    // write.open("/home/exbot/Desktop/OdomRAWdata.txt",ios::out|ios::app);
    // // write<<easting<<" "<<northing<<endl;
    // write<<setiosflags(ios::fixed)<<setprecision(7)<<s<<" x: "<<odom.pose.pose.position.x<<" y: "<<odom.pose.pose.position.y<<" z: "<< odom.pose.pose.orientation.z <<" w: "<< odom.pose.pose.orientation.w<<endl;
    // write.close();

    // Use ENU covariance to build XYZRPY covariance
    boost::array<double, 36> covariance = {{
    fix->position_covariance[0],
    fix->position_covariance[1],
    fix->position_covariance[2],
    0, 0, 0,
    fix->position_covariance[3],
    fix->position_covariance[4],
    fix->position_covariance[5],
    0, 0, 0,
    fix->position_covariance[6],
    fix->position_covariance[7],
    fix->position_covariance[8],
    0, 0, 0,
    0, 0, 0, rot_cov, 0, 0,
    0, 0, 0, 0, rot_cov, 0,
    0, 0, 0, 0, 0, rot_cov
    }};

    odom.pose.covariance = covariance;
   // printf("x= %f y= %f\n",odom.pose.pose.position.x, odom.pose.pose.position.y);
    odom_pub.publish(odom);
    loop_rate.sleep();
  }
}


int main (int argc, char **argv) 
{
  ros::init(argc, argv, "utm_odometry_node");
  ros::NodeHandle node,node1;
  ros::NodeHandle priv_node("~");

  priv_node.param<std::string>("frame_id", frame_id, "");//gps
  priv_node.param<std::string>("child_frame_id", child_frame_id, "base_footprint");
  priv_node.param<double>("rot_covariance", rot_cov, 99999.0);

  odom_pub = node.advertise<nav_msgs::Odometry>("odom",350);//odom (包含imu)

  ros::Subscriber fix_sub = node.subscribe("fix",350, callback);//10---1000
  // ros::Subscriber imu_sub = node1.subscribe("imu/data",100,imu_callback);/////gai 10--100

  // ros::Subscriber turtlebot_odomsub = node1.subscribe("odom1",10,turtlebotodom_callback);
  ros::spin();
}
