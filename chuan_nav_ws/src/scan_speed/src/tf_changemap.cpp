#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

int control=0;
double initeasting;
float du;
double initnorthing;

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

void odom_callback(const nav_msgs::OdometryConstPtr& msg)
{	
	tf::Quaternion q;
	// 获取偏航角
	float y1=getYitch(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
	float y2=0.9;
	du=y1*180.0/3.14159;
	q.setRPY(0, 0, y1);//0.8529 //转化到tf::Quaternion q 下
	printf("%fy d%f\n",y1,du);


	static  tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0) );  //当前机器人的位置（已经做过相对原点位置转换）
	//tf::Quaternion q;
	// q.setRPY(0, 0, -y1);
	//transform.setRotation( tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,msg->pose.pose.orientation.z, msg->pose.pose.orientation.w) );
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_footprint"));//odom
}


int main(int argc, char** argv){

ros::init(argc, argv, "robot_tf_publisher");

ros::NodeHandle n;
// ros::Rate rate(51.0);
ros::Subscriber odom_sub = n.subscribe("/odom",300,odom_callback);//odom 10--100

ros::spin();
return 0;
};
