可以直接放在ros空间下的src 下

serial_example.cc   官网提供 的字符串读写测试
serial_xx.cpp     串口函数测试


serial_ros.cpp 在ros下进行串口的读写操作。
运行  ： rosrun serial  serial_ros  /dev/ttyUSB0   9600

serial_ros_1.cpp   串口函数测试专用

serial_ros_xx.cpp  通过订阅'/chatter ' 和发布 '/read '的形式测试串口
运行  ： rosrun serial  serial_ros_xx 


