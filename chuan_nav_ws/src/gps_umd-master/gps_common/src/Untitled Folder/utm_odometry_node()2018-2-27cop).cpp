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
using namespace gps_common;
using namespace std;
static ros::Publisher odom_pub;
std::string frame_id, child_frame_id;
double rot_cov;
float du;
   double initeasting;
   double initnorthing;
int control=0;
float x_1,y_1,z_1,w_1,xv_1,yv_1,zv_1,twistx,twisty;
//covert to r p y

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


void callback(const sensor_msgs::NavSatFixConstPtr& fix) {
  if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
    ROS_INFO("No fix.");
    return;
  }
ros::Rate loop_rate(100);///////////////////////////////////xiugai  50---100
  if (fix->header.stamp == ros::Time(0)) {
    return;
  }
  ofstream write;
  double northing, easting;
  std::string zone;
  
  //printf("lat  %f\n  longitude   %f\n ",fix->latitude,fix->longitude);
  //cout<<"lat"<<fix->latitude<<"   longitude"<<fix->longitude<<endl;
  LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);
 // printf("\n ww1111为：\n %0.08lf\n,\n jj1111为：\n %0.08lf\n,",fix->latitude,fix->longitude);
 
 if(control==0)
{
  // initeasting=easting;jiantu 442313.864424
  // initnorthing=northing;   3438882.029674  113.58
	initeasting=442319.158232;  
   initnorthing= 3438871.471718;     
   control=1;
}
//printf("initlat  %f\n  longitude   %f\n ",initeasting, initnorthing);

 //printf("%feasting northing%f\n",easting-initeasting,northing-initnorthing);
/*
  write.open("/home/ubuntu/Desktop/GPSRAWdata.txt",ios::out|ios::app);
  // write<<easting<<" "<<northing<<endl;
 write<<setiosflags(ios::fixed)<<setprecision(7)<<fix->header.stamp<<" lat"<<fix->latitude<<"  log"<<fix->longitude<<endl;
  write.close(); 
*/

//printf("beicha %f\n  ",northing-initnorthing );
//printf("dongcha %f\n  ",easting-initeasting );
  //cout<<"east"<<easting<<" northing"<<northing<<endl;
  if (odom_pub) {
    nav_msgs::Odometry odom;
    odom.header.stamp = fix->header.stamp;

    if (frame_id.empty())
      odom.header.frame_id = fix->header.frame_id;
    else
      odom.header.frame_id = frame_id;

    odom.child_frame_id = child_frame_id;

    odom.pose.pose.position.x = easting-initeasting;
    odom.pose.pose.position.y = northing-initnorthing;
    odom.pose.pose.position.z = 0;//fix->altitude
    
    odom.pose.pose.orientation.x = x_1;
    odom.pose.pose.orientation.y = y_1;
    odom.pose.pose.orientation.z = z_1;//z_1
    odom.pose.pose.orientation.w = w_1;//w_1

   odom.twist.twist.linear.x=twistx;
     odom.twist.twist.angular.z=twisty;
    
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

    odom_pub.publish(odom);
    loop_rate.sleep();
  }
}

int main (int argc, char **argv) {
  ros::init(argc, argv, "utm_odometry_node");
  ros::NodeHandle node,node1;
  ros::NodeHandle priv_node("~");
  
  priv_node.param<std::string>("frame_id", frame_id, "");//gps
  priv_node.param<std::string>("child_frame_id", child_frame_id, "base_footprint");
  priv_node.param<double>("rot_covariance", rot_cov, 99999.0);

  odom_pub = node.advertise<nav_msgs::Odometry>("odom", 100);//odom

  ros::Subscriber fix_sub = node.subscribe("fix", 200, callback);//10---1000
  ros::Subscriber imu_sub = node1.subscribe("imu/data",100,imu_callback);/////gai 10--100

  // ros::Subscriber turtlebot_odomsub = node1.subscribe("odom1",10,turtlebotodom_callback);

  ros::spin();
}

