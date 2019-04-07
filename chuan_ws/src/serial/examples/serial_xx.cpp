/***
 * This example expects the serial port has a loopback on it.
 *
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());
 *    }
 *  }
 * </pre>
 */

#include <string>
#include <iostream>
#include <cstdio>

// OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "serial/serial.h"

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

void my_sleep(unsigned long milliseconds) {
#ifdef _WIN32
      Sleep(milliseconds); // 100 ms
#else
      usleep(milliseconds*1000); // 100 ms
#endif
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

  // Argument 1 is the serial port or enumerate flag   //第一个参数   /dev/ttyUSB0
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
#if defined(WIN32) && !defined(__MINGW32__)
  sscanf_s(argv[2], "%lu", &baud);
#else
  sscanf(argv[2], "%lu", &baud);
#endif

  // port, baudrate, timeout in milliseconds
  serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));

  cout << "Is the serial port open?";
  if(my_serial.isOpen())
    cout << " Yes." << endl;
  else
    cout << " No." << endl;

  // Get the Test string
  //char ask[3] = {0x11,0x12,0x13};
  char ask[3] = {11,12,13};
  int count = 0;
  int i=0;
  string test_string;
  if (argc == 4) {
    test_string = ask;
  } else {
    test_string = ask;
  }

  // Test the timeout, there should be 1 second between prints
  cout << "Timeout == 1000ms, asking for 1 more byte than written." << endl;
  while (count < 10)
  {
    size_t bytes_wrote = my_serial.write(test_string);

    string result = my_serial.read(test_string.length()+1);
    //string result = my_serial.read(3);

    // cout << "Iteration: " << count << ", Bytes written: ";
    // cout << bytes_wrote << ", Bytes read: ";
    // cout << ", String read: " << result << endl;

    for(i=0; i<3; i++)     
    {
    //  printf("%x\n", result[i]);
      printf("%d\n", result[i]);
    }
    count += 1;
  }


  return 0;
}

int main(int argc, char **argv) {
  try {
    return run(argc, argv);
  } catch (exception &e) {     //异常处理
    cerr << "Unhandled Exception: " << e.what() << endl;
  }
}
