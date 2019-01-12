// 小号 gps+imu 

#include "ros/time.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdlib.h>
#include<math.h>
#include <iostream>
#include <stdio.h>
#include <pthread.h>
#include <string.h>
#include <sys/stat.h>   /**/
#include <fcntl.h>      /*文件控制定义*/
#include <termios.h>    /*PPSIX终端控制定义*/
#include <errno.h>      /*错误号定义*/
using namespace std;
#include <set>
#include <iomanip>
//#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"

//ofstream write;
#include <iostream>
#include <string>
#include <algorithm>
#include <signal.h>
#include <fstream>
#include <sstream>

#define waitforstx1  1    // 检查包头1
#define waitforstx2  2     //检查包头2
#define waitformid  3      //检查包mid数据类型
//#define waitforlen  4     //检查数据长度79
#define waitfordata 4    // 提取数据
ros::Publisher fixdata,imu_data;
sensor_msgs::NavSatFix msg;
sensor_msgs::Imu    imu_msg;
float x_1,x_2,x_3,y_1,y_2,y_3,z_1,z_2,z_3,w_1,x_11,y_11,z_11;
float x_r,y_r,z_r;
double ww1,xx1,yy1,zz1,rr1,pp1,yy11;
//ros::NodeHandle n,n1;
ros::Time current_time;
ros::Time current_time1;
float covariance[9]={0};
float z_21=0;
float z_22;
int fd1;
int nread;
int control1=0;
float init_x;
int state=1;
float Hdecimal=0.0;
unsigned char data_buff[1024]={0};
int length=0;
int control=0;
//int for_i=0;
int i=0;
int lsize=0;
uint32_t laot_buff[12]={0};
long int lat_1,lot_1;
double lat_2,lot_2,ldt_2;
int speed_arr[] = { B38400, B19200, B9600, B4800, B2400, B1200, B300,
B38400, B19200, B9600, B4800, B2400, B1200, B300, };
unsigned char hexbyte[4]={0};
unsigned char hexbyte1[4]={0};
void pubfixdata(sensor_msgs::NavSatFix msg);
void pubimu_data(sensor_msgs::Imu msg);
// bzero(( unsigned char*)&hexbyte,4);
//bzero(( unsigned char*)&hexbyte1,4);

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

double getw(double r,double p,double y)
{

double w_w=	cos(r/2)*cos(p/2)*cos(y/2)+sin(r/2)*sin(p/2)*sin(y/2);
return w_w;
}

double getx(double r,double p,double y)
{

double x_x=	cos(r/2)*sin(p/2)*cos(y/2)+sin(r/2)*cos(p/2)*sin(y/2);
return x_x;
}

double gety(double r,double p,double y)
{

double y_y=	cos(r/2)*cos(p/2)*sin(y/2)-sin(r/2)*sin(p/2)*cos(y/2);
return y_y;
}

double getz(double r,double p,double y)
{

double z_z=	sin(r/2)*cos(p/2)*cos(y/2)-cos(r/2)*sin(p/2)*sin(y/2);
return z_z;
}
*/
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

double getw(double r,double p,double y)
{

double w_w=	cos(r/2)*cos(p/2)*cos(y/2)+sin(r/2)*sin(p/2)*sin(y/2);
return w_w;
}

double getx(double r,double p,double y)
{

double x_x=	sin(r/2)*cos(p/2)*cos(y/2)-cos(r/2)*sin(p/2)*sin(y/2);
return x_x;
}

double gety(double r,double p,double y)
{

double y_y=	cos(r/2)*sin(p/2)*cos(y/2)+sin(r/2)*cos(p/2)*sin(y/2);
return y_y;
}

double getz(double r,double p,double y)
{

double z_z=	cos(r/2)*cos(p/2)*sin(y/2)-sin(r/2)*sin(p/2)*cos(y/2);
return z_z;
}

int set_opt(int fd3,int nSpeed, int nBits, char nEvent, int nStop)   
{   //cout<<lat_1<<endl;
struct termios newtio,oldtio;   
/*保存测试现有串口参数设置，在这里如果串口号等出错，会有相关的出错信息*/   
if  ( tcgetattr( fd3,&oldtio)  !=  0) {    
perror("SetupSerial 1");  
printf("tcgetattr( fd3,&oldtio) -> %d\n",tcgetattr( fd3,&oldtio));   
return -1;   
}   
bzero( &newtio, sizeof( newtio ) );   
/*步骤一，设置字符大小*/   
newtio.c_cflag  |=  CLOCAL | CREAD;    
newtio.c_cflag &= ~CSIZE;    
/*设置停止位*/   
switch( nBits )   
{   
case 7:   
newtio.c_cflag |= CS7;   
break;   
case 8:   
newtio.c_cflag |= CS8;   
break;   
}   
/*设置奇偶校验位*/   
switch( nEvent )   
{   
case 'o':  
case 'O': //奇数   
newtio.c_cflag |= PARENB;   
newtio.c_cflag |= PARODD;   
newtio.c_iflag |= (INPCK | ISTRIP);   
break;   
case 'e':  
case 'E': //偶数   
newtio.c_iflag |= (INPCK | ISTRIP);   
newtio.c_cflag |= PARENB;   
newtio.c_cflag &= ~PARODD;   
break;  
case 'n':  
case 'N':  //无奇偶校验位   
newtio.c_cflag &= ~PARENB;   
break;  
default:  
break;  
}   
/*设置波特率*/   
switch( nSpeed )   
{   
case 2400:   
cfsetispeed(&newtio, B2400);   
cfsetospeed(&newtio, B2400);   
break;   
case 4800:   
cfsetispeed(&newtio, B4800);   
cfsetospeed(&newtio, B4800);   
break;   
case 9600:   
cfsetispeed(&newtio, B9600);   
cfsetospeed(&newtio, B9600);   
break;   
case 115200:   
cfsetispeed(&newtio, B115200);   
cfsetospeed(&newtio, B115200);   
break;   
case 460800:   
cfsetispeed(&newtio, B460800);   
cfsetospeed(&newtio, B460800);   
break;   
default:   
cfsetispeed(&newtio, B9600);   
cfsetospeed(&newtio, B9600);   
break;   
}   
/*设置停止位*/   
if( nStop == 1 )   
newtio.c_cflag &=  ~CSTOPB;   
else if ( nStop == 2 )   
newtio.c_cflag |=  CSTOPB;   
/*设置等待时间和最小接收字符*/   
newtio.c_cc[VTIME]  = 0;   
newtio.c_cc[VMIN] = 0;   
/*处理未接收字符*/   
tcflush(fd3,TCIFLUSH);   
/*激活新配置*/   
if((tcsetattr(fd3,TCSANOW,&newtio))!=0)   
{   
perror("com set error");   
return -1;   
}   
printf("set done!\n");   
return 0;   
}   

/**封装imu话题数据类型：senser_msgs/Imu

*/

sensor_msgs::Imu imudata()
{
current_time1=ros::Time::now();
imu_msg.header.stamp=current_time1;
imu_msg.header.frame_id="imu";
imu_msg.orientation.x=xx1;
imu_msg.orientation.y=yy1;
imu_msg.orientation.z=zz1;
imu_msg.orientation.w=ww1;
for(int k=0;k<9;k++)
{

imu_msg.orientation_covariance[k]=covariance[k]	;
}
imu_msg.angular_velocity.x=x_2;
imu_msg.angular_velocity.x=y_2;
imu_msg.angular_velocity.x=z_2;
for(int m=0;m<9;m++)
imu_msg.angular_velocity_covariance[m]=0;
imu_msg.linear_acceleration.x=x_3;
imu_msg.linear_acceleration.x=y_3;
imu_msg.linear_acceleration.x=z_3;
return imu_msg;	
}


/**封装fix话题数据类型：senser_msgs/NavSatfix

*/
sensor_msgs::NavSatFix datamsg()
{
//sensor_msgs::NavSatFix msg;
current_time=ros::Time::now();
msg.header.stamp=current_time;
msg.header.frame_id="odom";
//msg.header.frame_id="gps";	
msg.status.status=1;	
msg.status.service=1;	
msg.latitude=lat_2;		
msg.longitude=lot_2;
msg.altitude=ldt_2;
msg.position_covariance[0]=0.4;
msg.position_covariance[4]=0.4;
msg.position_covariance[8]=0.4;
msg.position_covariance_type=1;
return msg;
}

/**发布话题fix

*/
/*
void pubfixdata(sensor_msgs::NavSatFix msg)
{

fixdata=n.advertise<sensor_msgs::NavSatFix>("fix/gps",10);
fixdata.publish(msg);

}
*/
/**发布话题fix

*/
/*
void pubimu_data(sensor_msgs::Imu msg)
{

imu_data=n1.advertise<sensor_msgs::Imu>("imu/data",10);
imu_data.publish(msg);

}
*/			

/**进制转换

*/

float Hex_To_Decimal(unsigned char *Byte,int num)//十六进制到浮点数
{
char cByte[4];
for (int i=0;i<num;i++)
{
cByte[i] = Byte[i];
}

float pfValue=*(float*)&cByte;
return pfValue;
}

/**
*@breif 提取经纬度
*/
long int recevelat(int j)
{   long int lat1;
for(int for_j=j;for_j<=j+12;for_j++)
{
laot_buff[for_j-j]=data_buff[for_j];
//printf("laot——buff==%x\n",laot_buff[for_j-j]);
}
//laot_buff[0]=0xff;laot_buff[1]=0xff;laot_buff[2]=0xec;laot_buff[3]=0x73;
//printf("lat111%x\n",laot_buff[0]);
lat1=(laot_buff[0]<<24)+(laot_buff[1]<<16)+(laot_buff[2]<<8)+(laot_buff[3]);
for(int i;i<4;i++)	
//printf("%x\n",laot_buff[i]);
//printf("lat=%ld\n",lat1);
return lat1;		
}
/**
*@breif 提取yaw：
*/

float receveyaw(int k)
{
for(int for_i=k;for_i<=k+3;for_i++)
{
hexbyte[for_i-k]=data_buff[for_i];
//printf("%x\n",data_buff[for_i]);
}
for(int j=3;j>=0;j--)
hexbyte1[3-j]=hexbyte[j];
Hdecimal=Hex_To_Decimal(hexbyte,sizeof(hexbyte));//十六进制转换为浮点数
//printf("\n yaw为：\n %f\n",Hdecimal);
return Hdecimal;
}			
/**
*@breif 打开串口
*/
int OpenDev(char *Dev)
{
int	fd2 = open( Dev, O_RDWR );         //| O_NOCTTY | O_NDELAY
if (-1 == fd2)
{ /*设置数据位数*/
perror("Open Serial Port Fail111111111111111");
perror("Open Serial Port Fail");
return -1;
}
else
printf("Open Serial Port Success\n");
return fd2;

}


void  gps_control_callback()
{
	int counter=0;
	float z_22=10;
	//ofstream write;
	unsigned char temp_buff[1024];
	// unsigned char hexbyte[4];
	// unsigned char hexbyte1[4];
	char *dev ="/dev/ttyUSB0";
	bzero(( unsigned char*)&temp_buff,1024);
	// bzero(( unsigned char*)&hexbyte,4);
	//bzero(( unsigned char*)&hexbyte1,4);
	fd1 = OpenDev(dev);
	set_opt(fd1, 115200, 8, 'N', 1);
while(ros::ok())
{
	if(control==0)
	{
		if ((nread = read(fd1,temp_buff,1)) > 0)
		{	 
		switch(state)
		{
		case waitforstx1:
		if(temp_buff[0]==0xfa)         // if(temp_buff[0]==0x55)
		state=waitforstx2;
		else
		state=waitforstx1;
		break;

		case waitforstx2:
		if(temp_buff[0]==0xff)         // if(temp_buff[0]==0x55)
		state=waitformid;
		else
		state=waitforstx1;
		break;
		case waitformid:
		if(temp_buff[0]==0x32)         // if(temp_buff[0]==0x55)
		state=waitfordata;
		else
		state=waitforstx1;
		break;
		case waitfordata:
		if(length==lsize+1)
		{
		state=1;control=1;length=0;
		}
		else
		{
		data_buff[length++]=temp_buff[0];
		i++;
		lsize= data_buff[0];
		state=4; control=0; 
		// printf("length=%d\n",lsize);        
		}
		break;
		}
		}
	}
	if(control==1)
	{ 
		lat_2=recevelat(13)/10000000.0;    //   48 get 经纬度和高度53   lat=纬度 lot=经度  8
		lot_2=recevelat(17)/10000000.0;       //52                              //      12
		//cout<<"lat2="<<lat_2<<"  lot2="<<lot_2<<std::endl;
		ldt_2=recevelat(21)/10000000.0;                                           //16
		if(control1==0)
		{
			init_x=receveyaw(9);  //89
			control1=1;
		}
		//  printf(" yy111为：\n %f\n",init_x);
		//get r y p rate 得到滚动角，航向角，俯仰角，
		// y_1=receveyaw(1)/180*3.141592;//81
		// x_1=receveyaw(5)/180*3.141592;  //85
		z_1=receveyaw(9);   //yaw  航向角 89
		// cout<<" y_1:  "<<y_1 <<endl;
		// cout<<" x_1: "<<x_1 <<endl;
		//cout<<" 航向角："<<z_1<<endl;
		if(init_x<0)
		{      
			if(-180<z_1&&z_1<=0)
			{ 
			z_21=z_1-init_x;
			// printf(" yy111为：\n %f\n",z_22);	
			//cout<<"5555"<<std::endl;	
			}
			if(180+init_x<z_1&&z_1<=180)
			z_21=-180-(180-z_1)-init_x;
			if(0<z_1&&z_1<180+init_x)
			z_21=z_1-init_x;
		}
		if(init_x>0)
		{      
			if(init_x<z_1&&z_1<=180)
			{ 
			z_21=z_1-init_x;
			// printf(" yy111为：\n %f\n",z_22);	
			//cout<<"5555"<<std::endl;	
			}
			if(-180<z_1&&z_1<=-180+init_x)
			z_21=180+(180+z_1)-init_x;
			if(-180+init_x<z_1&&z_1<init_x)
			z_21=z_1-init_x;
		}
		z_22=z_21/180*3.141592;
		//cout<<" yaw du=  "<<z_22<<" fudu=  "<<z_21<<std::endl; 
			//std::cout<<"  "<<z_21<<"   "<<z_22<<std::endl: 
		//printf("\n ww11111111为：\n %f\n,\n xx11111111为：\n %f\n,\n yy111为：\n %f\n",x_1*180/3.14,y_1*180/3.14,z_21);
		ww1=getw(0,0,z_22);             //变换得到四元素角
		xx1=getx(0,0,z_22);
		yy1=gety(0,0,z_22);
		zz1=getz(0,0,z_22);
		//printf("\n ww1为：\n %f\n,\n xx1为：\n %f\n,\n yy1为：\n %f\n,\n zz1为：\n %f\n",ww1,xx1,yy1,zz1);

		//rr1= getRoll(xx1,yy1,zz1,ww1)*180/3.14159;
		// pp1= getPaw(xx1,yy1,zz1,ww1)*180/3.14159;
		//   yy11= getYitch(xx1,yy1,zz1,ww1)*180/3.141592;
		// printf("\n rr1为：\n %f\n,\n pp1为：\n %f\n,\n yy11为：\n %f\n,",rr1,pp1,yy11);

		//x_r=receveyaw(9);             //get x y z rate 得到x y z 轴速度
		//y_r=receveyaw(13);
		//z_r=receveyaw(17);

		//x_11=receveyaw(33);             //get n u e rate  得到北 天，东 速度
		//y_11=receveyaw(37);
		//z_11=receveyaw(41);

		// w_1=receveyaw(93);
		sensor_msgs::NavSatFix msg1=datamsg();
		sensor_msgs::Imu  msg2=imudata();
		pubfixdata(msg1);
		pubimu_data(msg2);
		control=0;//length=0;

		//write.open("/home/jie/Desktop/data15.txt",ios::out|ios::app);
		// write<<setprecision(8)<<x_1<<" "<<y_1<<" "<<endl;
		//write.close(); 
	}  
}
}

int main(int argc,char** argv)
{      
ros::init(argc, argv, "gps");
ros::NodeHandle n,n1;
gps_control_callback();

}
/**发布话题fix

*/

void pubfixdata(sensor_msgs::NavSatFix msg)
{
ros::NodeHandle n;
//ros::Rate rate(11.0);
fixdata=n.advertise<sensor_msgs::NavSatFix>("fix",10);
fixdata.publish(msg);
// rate.sleep();

}

/**发布话题fix

*/

void pubimu_data(sensor_msgs::Imu msg)
{
ros::NodeHandle n1;
//ros::Rate rate(12.0);
imu_data=n1.advertise<sensor_msgs::Imu>("imu/data",10);
imu_data.publish(msg);
// rate.sleep();
}


