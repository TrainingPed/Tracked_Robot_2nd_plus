#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "stdio.h"  
static const std::string OPENCV_WINDOW = "Image window";
cv::Size videoSize;
cv::VideoWriter writer;
int Frame=0;
std::ostringstream stringStream;
std::string filename;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
  public:

  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    // image_sub_ = it_.subscribe("/zed/rgb/image_raw_color", 1,
    //   &ImageConverter::imageCb, this);
        image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1,&ImageConverter::imageCb, this);

    cv::namedWindow(OPENCV_WINDOW,CV_WINDOW_AUTOSIZE);
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
//       cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    // videoSize = cv::Size(cv_ptr->image.cols ,cv_ptr->image.rows);
    cv::Mat img2,dst1;
    img2=cv_ptr->image;
    // Draw an example circle on the video stream
    // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //   cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
    // if(Frame!=0)
    // {
    //   writer.write(cv_ptr->image);
    //   std::cout<<"resolution:"<<cv_ptr->image.cols<<"x"<<cv_ptr->image.rows<<"\n";
    // }


//    img2.convertTo(dst1,CV_8UC1,255/10,0);
    img2.convertTo(dst1,CV_8UC1,1,0);
//    printf("show\n");
    cv::imshow(OPENCV_WINDOW, dst1);
//    cv::imshow(OPENCV_WINDOW, img2);
    // Update GUI Window
    
    cv::waitKey(3);
    Frame++;


  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  
   std::cout<<"Please enter filename ...\n";
   std::getline(std::cin, filename);
     stringStream<<"/home/cirlab/Videos/"<<filename<<".avi";//save as avi
     filename = stringStream.str();
  
  ImageConverter ic;
  ros::spinOnce();//first time get rows and cols

  writer.open(filename, CV_FOURCC('M', 'J', 'P', 'G'), 30, videoSize);
//   Create an RGBA sl::Mat object


  while(ros::ok())
  {
    ros::spinOnce();
    if(cv::waitKey(33) == 27)//press esc to escape
    { 
       break;
    }
  }
  return 0;
}
