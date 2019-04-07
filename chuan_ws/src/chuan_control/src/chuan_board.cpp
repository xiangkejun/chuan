//xx  20190407


 
#include "serial/serial.h"

#include<unistd.h>    //延时  

std::string rec_buf;

int main (int argc,char* argv[])
{
  
 // unsigned char rcv_buf[10];
  // 0x0d   "/r" 字符
  // 0x0a   "/n" 字符
  unsigned char send_buf[10]={0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x0d,0x0a};

  std::string port("/dev/ttyUSB0");
  unsigned long baud = 115200;
  serial::Serial my_serial(port,baud,serial::Timeout::simpleTimeout(1000)); //配置串口

  while(1) 
  {
      my_serial.write(send_buf,10);
       sleep(1);  //10ms


     rec_buf = my_serial.readline(10,"\n");
     const char *receive_data = rec_buf.data();
     if(rec_buf.length() == 10)
     {
          
      printf("---------%d\n",rec_buf.length());
      printf("1=%x\n",receive_data[0]);
      printf("2=%x\n",receive_data[1]);
		  printf("3=%x\n",receive_data[2]);
		  printf("4=%x\n",receive_data[3]); 
      printf("5=%x\n",receive_data[4]);
      printf("6=%x\n",receive_data[5]);
		  printf("7=%x\n",receive_data[6]);
		  printf("8=%x\n",receive_data[7]); 
      printf("9=%x\n",receive_data[8]); 
      printf("10=%x\n",receive_data[9]);      

     }
   //  if(len==4 || rcv_buf[0]== 1 || rcv_buf[3]== 4)	
   //   {


    //  }
      
    //   for(i=0;i<3;i++)
    //  {
    //    hexbyte[i] = rcv_buf[i];
    //  }
    //   Hdecimal=Hex_To_Decimal(hexbyte,sizeof(hexbyte));//十六进制转换为浮点数
    //   printf("FUFU %f\n",Hdecimal);
    }
  return 0;
}
