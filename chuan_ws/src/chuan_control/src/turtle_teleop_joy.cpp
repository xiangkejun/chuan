// 添加rpi3 引脚 做电源控制

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>



#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>


int state =1;
const int yaogan=2;
const int key_A =3;
const int key_B =4;
const int key_X =5;
const int key_Y =6;
const int key_back_start =7;
const int key_stop =8;


int key_0,key_1,key_2,key_3,key_4,key_5,key_6, key_7,key_8,key_9;
int state_power=0;
void *state_joy( void *arg )
{
  while(ros::ok())
  {
        if(key_0 ==0 && key_1==0 && key_2==0 && key_3==0 && key_8 == 0 || key_9 == 0)    //ABXY  guizheng
      {
          state = yaogan; 
      }
      
      if(key_4 == 1  || key_5 == 1)   // 刹车
        {
        state = key_stop;
        }
      if(key_8 == 1 || key_9 == 1)   //归正
        {
            state = key_back_start;
        }
      
    //  按键遥控  ABXY
      if(key_2 == 1)     // A
      {
      state = key_A;
      }
      if(key_0 == 1)     // Y
      {
        state =key_Y;
      }
      if(key_3 == 1)     // x
      {
        state = key_X;
      }
      if(key_1 == 1)     // B
      {
        state = key_B;
      }
  }
}

void create_all_thread(void)
{
	pthread_t thread_joy;
	if( (pthread_create( &thread_joy , NULL , state_joy, NULL )) != 0 )
	{
		perror("Create the thread_joy fail");
		exit( 1 );
	}	
}


class TeleopTurtle
{
public:
  TeleopTurtle();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

};



TeleopTurtle::TeleopTurtle():
  linear_(1),
  angular_(2)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);


  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);


  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopTurtle::joyCallback, this);

}


void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  
  geometry_msgs::Twist twist;

  key_0 = joy->buttons[0];
  key_1 = joy->buttons[1];
  key_2 = joy->buttons[2];
  key_3 = joy->buttons[3];
  key_4 = joy->buttons[4];
  key_5 = joy->buttons[5];
  key_6 = joy->buttons[6];
  key_7 = joy->buttons[7];
  key_8 = joy->buttons[8];
  key_9 = joy->buttons[9];
  usleep(1000);

   switch(state)
   {
       case yaogan:
            twist.angular.z = a_scale_*joy->axes[angular_];
            twist.linear.x = l_scale_*joy->axes[linear_];
            break;
       case key_A:
           twist.linear.x = -1 ;
           twist.angular.z = 0 ;

            break;
        case key_B:
            twist.linear.x = 0 ;
            twist.angular.z = -1 ;

            break;
        case key_X:
            twist.linear.x = 0 ;
            twist.angular.z = 1 ;

            break;
        case key_Y:
            twist.linear.x = 1;
            twist.angular.z = 0 ;

            break;
        case key_back_start:   //归正
            twist.angular.z = 2.5;
            twist.linear.x = 2.5;

            break;
        case key_stop:
            twist.angular.z = 0;
            twist.linear.x = 0;
            break;
        // case key_power_on:     
        //    digitalWrite (LED, HIGH) ; 
        //     break;
        // case key_power_off:
        //    digitalWrite (LED, LOW) ; 
        //     break;
        default:
             
            break;
   }


//********************
  vel_pub_.publish(twist);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  // digitalWrite (LED, LOW) ;   //start
  create_all_thread(); 
  TeleopTurtle teleop_turtle;

 // pthread_exit(NULL);
  ros::spin();

}
