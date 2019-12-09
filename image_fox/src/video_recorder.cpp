#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "stdio.h"
#include <time.h>

static const std::string OPENCV_WINDOW = "Image window";
cv::Size videoSize;
//cv::VideoWriter writer;
float Frame=0;
float Frame_old=0;
float dt=0;
std::ostringstream stringStream;
std::string filename;

class ImageConverter
{
  ros::NodeHandle nh_;

  
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  
  
  public:

  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/zed/rgb/image_rect_color", 200,
      &ImageConverter::imageCb, this);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    videoSize = cv::Size(cv_ptr->image.cols ,cv_ptr->image.rows);
    // Draw an example circle on the video stream
    // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //   cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
    if(Frame!=0)
    {
     // writer.write(cv_ptr->image);
    }
     cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    // Update GUI Window
    
    //  cv::waitKey(3);
    Frame++;


  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_recorder");
  
  std::cout<<"Please enter filename ...\n";
  std::getline(std::cin, filename);
	stringStream<<"/home/nvidia/Videos/"<<filename<<".avi";//save as avi
	filename = stringStream.str();
  
  ImageConverter ic;
  
  ros::Time current_time, last_time;

  ros::spinOnce();//first time get video size

 // writer.open(filename, CV_FOURCC('M', 'J', 'P', 'G'), 30, videoSize);
  ros::Rate r(30);
  std::cout<<"start record video ...\n";
  while(ros::ok())
  {
    last_time=ros::Time::now();
    ros::spinOnce();
    if(cv::waitKey(33) == 27)//press esc to escape
    { 
       break;
    }
    current_time=ros::Time::now();
    dt=(current_time-last_time).toSec();
    float fps=(Frame-Frame_old)/dt;
    std::cout<<"True Fps ="<<fps<<"\n";
    Frame_old=Frame;
    // r.sleep();
  }
  return 0;
}
