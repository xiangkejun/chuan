#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include "serial/serial.h"

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
string result;
void my_sleep(unsigned long milliseconds)
{
      usleep(milliseconds*1000); // 100 ms
}

void enumerate_ports()
{
    vector<serial::PortInfo> devices_found = serial::list_ports();

    vector<serial::PortInfo>::iterator iter = devices_found.begin();

    while( iter != devices_found.end() )
    {
        serial::PortInfo device = *iter++;

        printf( "(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
     device.hardware_id.c_str() );
    }
}

void print_usage()
{
    cerr << "Usage: test_serial {-e|<serial port address>} ";
    cerr << "<baudrate> [test string]" << endl;
}

int run(int argc, char **argv)
{
  if(argc < 2) {
      print_usage();
    return 0;
  }

  // Argument 1 is the serial port or enumerate flag
  string port(argv[1]);

  if( port == "-e" ) {
      enumerate_ports();
      return 0;
  }
  else if( argc < 3 ) {
      print_usage();
      return 1;
  }

  // Argument 2 is the baudrate
  unsigned long baud = 0;
  sscanf(argv[2], "%lu", &baud);
  // port, baudrate, timeout in milliseconds
  serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));
  /*cout << "Is the serial port open?";
  if(my_serial.isOpen())
    cout << " Yes." << endl;
  else
    cout << " No." << endl;
   */
  // Get the Test string
  int count = 0;
  string test_string;
  if (argc == 4) {
    test_string = argv[3];
  } else {
    test_string = "Testing.";
  }
  // Test the timeout at 250ms, but asking exactly for what was written
    my_serial.setTimeout(serial::Timeout::max(), 50, 0, 50, 0);
    size_t bytes_wrote = my_serial.write(test_string);
    result = my_serial.read(test_string.length()+1);
    cout << result.length() << ", String read: " << result << endl;
  return 0;
}

int main(int argc, char **argv) 
{
 ros::init(argc,argv,"serial_example");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("uart1",1000);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    std_msgs::String msg;
    std::stringstream ss;
    run(argc, argv);
    ss<<result ;
    msg.data=ss.str();
    chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0 ;
}
