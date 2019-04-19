#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <stdio.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);
  
  // std::string video_name = "rtsp://admin:txz123456@192.168.1.66:554/mjpeg/ch1/sub/av_stream";
  // int camera_index = 0;
  
  // cv::VideoWriter writer("/home/andyoyo/calib_ws/src/ros_cv/data/test.avi",CV_FOURCC('M','J','P','G'),25.0,cv::Size(640,480));

  cv::VideoCapture cap;
//  cap.open(video_name);
  cap.open(1);

 cap.set(CV_CAP_PROP_FRAME_WIDTH,640); // 1600x1200,960x720,640x480,320x240
 cap.set(CV_CAP_PROP_FRAME_HEIGHT,480); 

  std::string img_name;
  if(!cap.isOpened())
  {
    std::cout<<"can not open camera!"<<std::endl;
    return -1;
  }
  cv::Mat frame;
  ros::Rate loop_rate(30);
  
  while(nh.ok())
  {
    cap>>frame;
    if(frame.empty())
    {
      std::cout<<"frame is empty!"<<std::endl;
      return -1;
    }
   // writer<<frame;
    cv::imshow("video",frame);
    if(cv::waitKey(10)==27)
    {
      std::cout<<"ESC!"<<std::endl;
      return -1;
    }

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg(); 
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

}
