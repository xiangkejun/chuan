//xx  20190407
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "serial/serial.h"
#include<unistd.h>    //延时  
float linear_vel=0.0,angular_vel=0.0;
union floatData  //用于char数组与float之间的转换
{
  float f;  
  unsigned char data[4];
}chaun_linear_vel_send,chuan_angular_vel_send,chaun_linear_vel_rec,chuan_angular_vel_rec;

  // 0x0d   "/r" 字符
  // 0x0a   "/n" 字符
  //{0x0,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x0d,0x0a};
unsigned char send_buf[10]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0d,0x0a};

void chuan_vel_callback(const geometry_msgs::Twist::ConstPtr& cmd )
{
    std::string port("/dev/ttyUSB0");
    unsigned long baud = 115200;
    serial::Serial my_serial(port,baud,serial::Timeout::simpleTimeout(1000)); //配置串口

   // 发送
      linear_vel = cmd->linear.x;
      angular_vel = cmd->angular.z;
      chaun_linear_vel_send.f = linear_vel * 1000; // 放大1000 倍
      chuan_angular_vel_send.f = angular_vel * 1000;
      std::cout <<"chaun_linear_vel=" <<chaun_linear_vel_send.f/1000<<std::endl;
      std::cout<<"chuan_angular_vel="<<chuan_angular_vel_send.f/1000<<std::endl;
      for(int i=0; i<4; i++)
      {
        send_buf[i] = chaun_linear_vel_send.data[i];
        send_buf[i+4] = chuan_angular_vel_send.data[i];
      }
      my_serial.write(send_buf,10);
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "chuan_board");
   ros::NodeHandle n;
  //  ros::Subscriber sub = n.subscribe("/cmd_vel_mux/input/teleop", 1, keyboard_callback);
   ros::Subscriber sub = n.subscribe("/mobile_base/commands/velocity", 1, chuan_vel_callback);

    ros::spin();

  return 0;
}
