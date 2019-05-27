#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
 
int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle node;
 
  tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  ros::Rate rate(100);//node.ok()41
  while (ros::ok())
  {
    transform.setOrigin( tf::Vector3(0.0,0.0,0.0) );
      //transform.setRotation( tf::Quaternion(0, 0, 0, 1) );-1.1,-1.4, 0.0
    
    // 130 = 2.267  140 = 2.442  145 = 2.529  150 = 2.6167
    q.setRPY(0, 0, 2.529 );//-0.035  -0.175 1.5708  -0.8529 
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "odom"));   //odom相对于map是不动的(理想情况)
    rate.sleep();
  }
  return 0;
};

