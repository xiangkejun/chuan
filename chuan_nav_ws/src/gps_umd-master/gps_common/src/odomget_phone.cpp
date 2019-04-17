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
#include <tf/tf.h>

#include <stdio.h>
#include<cstdlib>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <iomanip>
using namespace gps_common;
using namespace std;
static ros::Publisher odom_pub;
std::string frame_id, child_frame_id;
double rot_cov;
double initeasting;
double initnorthing;
int control=0;
float o_x,o_y,o_z,o_w,xv_1,yv_1,zv_1,twist_linear_x,twist_linear_y,twist_ang_z;
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


double getYaw(double x,double y,double z, double w)
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
ros::Time current_time,last_time;
geometry_msgs::Quaternion orientation;
void imu_callback(const sensor_msgs::ImuConstPtr& imu)
{
    current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();

    // o_x = imu->orientation.x;
    // o_y = imu->orientation.y;
    // o_z = imu->orientation.z;
    // o_w = imu->orientation.w;
    // float yaw= getYitch( o_x,o_y,o_z, o_w);
    orientation = imu->orientation;
    tf::Matrix3x3 mat(tf::Quaternion(orientation.x,orientation.y,orientation.z,orientation.w));
    double yaw,pitch,roll,du_yaw,du_pitch,du_roll;
    mat.getEulerYPR(yaw,pitch,roll);

    du_yaw=yaw*180.0/3.1415;
    du_pitch=pitch*180/3.1415;
    du_roll=roll*180/3.1415;
    // cout<<"yaw="<<yaw<<"du_yaw="<<du_yaw<<"du_pitch="<<du_pitch<<"du_roll="<<du_roll<<endl;
    // printf("yaw=%f du=%f\n",yaw,du);	
    // twistx=sqrt(imu->linear_acceleration.x*imu->linear_acceleration.x+imu->linear_acceleration.z*imu->linear_acceleration.z);
    // twisty=imu->angular_velocity.y/180*3.1415926;
    twist_linear_x = imu->linear_acceleration.x * dt;
    twist_linear_y = imu->linear_acceleration.y * dt;
    twist_ang_z = imu ->angular_velocity.z;
    // printf("%fy d%f\n",yaw,du);	
    last_time = ros::Time::now();
    ros::Rate loop_rate(50);    
}

int k=0;
double weidu_sum = 0,jingdu_sum = 0,weidu,jingdu, northing, easting;

void callback(const sensor_msgs::NavSatFixConstPtr& fix) 
{
  if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) 
  {
    ROS_INFO("No fix.");
    return;
  }
  //float yaw= getYitch( x_1,y_1,z_1, w_1);

//   twistx=sqrt(fix->position_covariance[6]*fix->position_covariance[6]+fix->position_covariance[8]*fix->position_covariance[8]); //（（北速度平方+东速度平方）开方）
//   twisty=fix->position_covariance[5]/180*3.1415926; //y轴角速度
  ros::Rate loop_rate(50);///////////////////////////////////xiugai  50---100
  if (fix->header.stamp == ros::Time(0)) 
  {
    return;
  }
  ofstream write;
  std::string zone;
  weidu = fix->latitude;
  jingdu = fix->longitude;

//  k++;
//  weidu_sum = weidu_sum + fix->latitude;
//  jingdu_sum = jingdu_sum + fix->longitude;
//  if(k == 10)
//  {
//      k = 0;
//     weidu = weidu_sum/10;
//     jingdu = jingdu_sum/10;
//     weidu_sum =0;
//     jingdu_sum=0;
//     LLtoUTM(weidu, jingdu, northing, easting, zone);     //gps 数据格式转换
//     printf(" weidu=  %0.10lf jingdu=  %0.10lf\n",weidu,jingdu);

//  }

//   cout<<"weidu= "<<weidu<<"jingdu= "<<jingdu<<endl;
 //weidu=  21.3099405405 jingdu=  -157.8900592105

// latitude: 21.3099474383
// longitude: -157.890073665
 LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);     //gps 数据格式转换
    // LLtoUTM(weidu, jingdu, northing, easting, zone);     //gps 数据格式转换

//  LLtoUTM(21.3099405405  ,-157.8900592105, northing, easting, zone);     //gps 数据格式转换
 printf(" weidu=  %0.10lf jingdu=  %0.10lf\n",weidu,jingdu);
  // printf(" northing=  %0.10lf easting=  %0.10lf\n",northing,easting);
  if(control==0)    //对initeasting   initnorthing只赋一次初值
  {
    initeasting = 471661.4778352198;   //转换后的初始值
    initnorthing= 3489136.5319788991  ;    //3438861.211899
    control=1;

 //northing=  3489140.78025554 easting=  471661.74505490
//  northing=  2356855.4970923183 easting=  615120.3465072220
//  northing=  2356857.6118143634 easting=  615116.1169638692
//  northing=  3489136.5319788991 easting=  471661.4778352198


  }

  if (odom_pub) 
  {
    nav_msgs::Odometry odom;
    odom.header.stamp = fix->header.stamp;
    odom.header.frame_id = "odom";

    odom.child_frame_id = child_frame_id;
    odom.pose.pose.position.x = easting-initeasting;
    odom.pose.pose.position.y = northing-initnorthing;     //算相对原点的位置
    // odom.pose.pose.position.x = 0;
    // odom.pose.pose.position.y = 0;  //机器人处于地图原点
    odom.pose.pose.position.z = 0;//fix->altitude

    odom.pose.pose.orientation.x = orientation.x;
    odom.pose.pose.orientation.y = orientation.y;
    odom.pose.pose.orientation.z = orientation.z;
    odom.pose.pose.orientation.w = orientation.w;//  yaw的四元数形式 imu

    odom.twist.twist.linear.x=twist_linear_x;   //（（北速度平方+东速度平方）开方）
    odom.twist.twist.linear.y=twist_linear_y;
    odom.twist.twist.angular.z=twist_ang_z;  //y轴角速度  即正前方的角速度
   // timeco_dt();   //获取本地时间 存在  s 中
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

  odom_pub = node.advertise<nav_msgs::Odometry>("odom",10);//odom (包含imu)

  current_time = ros::Time::now();   // 用于dt
   last_time = ros::Time::now();
  
  ros::Subscriber fix_sub = node.subscribe("/android/fix",10, callback);//10---1000
   ros::Subscriber imu_sub = node1.subscribe("/android/imu",10,imu_callback);/////gai 10--100

  // ros::Subscriber turtlebot_odomsub = node1.subscribe("odom1",10,turtlebotodom_callback);
  ros::spin();
}
