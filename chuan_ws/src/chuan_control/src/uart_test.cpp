//xx  20190407

#include "serial/serial.h"

#include<unistd.h>    //延时  
#include <iostream>

std::string rec_buf;
float linear_vel=0.2,angular_vel=0.5;
union floatData  //用于char数组与float之间的转换
{
  float f;  
  unsigned char data[4];
}chaun_linear_vel_send,chuan_angular_vel_send,chaun_linear_vel_rec,chuan_angular_vel_rec;


int main (int argc,char* argv[])
{
  // 0x0d   "/r" 字符
  // 0x0a   "/n" 字符
  //{0x0,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x0d,0x0a};

  unsigned char send_buf[10]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0d,0x0a};

  std::string port("/dev/ttyUSB0");
  unsigned long baud = 115200;
  serial::Serial my_serial(port,baud,serial::Timeout::simpleTimeout(1000)); //配置串口

  while(1) 
  {  

    // 发送
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
       sleep(1);  //1s

   std::cout<<"-------------------"<<std::endl;

   //接收
     rec_buf = my_serial.readline(10,"\n");
     const char *receive_data = rec_buf.data();
     if(rec_buf.length() == 10)
     {
        for(int i=0; i<4; i++)
        {
            chaun_linear_vel_rec.data[i] = rec_buf[i];
            chuan_angular_vel_rec.data[i] = rec_buf[i+4];
        }
        std::cout<<"chaun_linear_vel_rec="<<chaun_linear_vel_rec.f/1000<<std::endl;
        std::cout<<"chuan_angular_vel_rec="<<chuan_angular_vel_rec.f/1000<<std::endl;    

     }

    }
  return 0;
}
