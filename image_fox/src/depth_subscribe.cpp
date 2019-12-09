#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "stdio.h"
#include <vector>
#include <fstream>
#include <iostream>
#define Col 640 //642(640)      can be devided by 3
#define Row 480 //342(340)      can be devided by 3

static const std::string OPENCV_WINDOW = "Image window";
//cv::Size videoSize;
cv::Mat img1(Col,Row,CV_32FC1);//save the depth image


int Frame=0;
int x,y=0;
std::ostringstream stringStream;
std::string filename;
std::vector <float> depth_data;
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
    image_sub_ = it_.subscribe("/camera/depth/image_rect", 1,&ImageConverter::imageCb, this);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    int px11=15;
    int px1=0,py1=0;
    float t1=0.2;
    int t2_o=8000;
    int t2_c=500;
    int px=642;
    int py=342;
    int px2=px1+px, py2=py1+py;
    bool Crop=0;
    nh_.getParam("px1",px11);
    nh_.getParam("py1",py1);
    nh_.getParam("px",px);
    nh_.getParam("py",py);
    nh_.getParam("t1",t1);
    nh_.getParam("t2_c",t2_c);//center blocks' threshold
    nh_.getParam("t2_o",t2_o);//other blocks' threshold
    nh_.getParam("Crop",Crop);//other blocks' threshold
    px1=px11;
    
    
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    x=(cv_ptr->image.cols);
    y=(cv_ptr->image.rows);
    // std::cout<<"x="<<x<<"y="<<y<<"\n";
    //depth_data = cv_ptr->image.at<float>(y/2,x/2);
    //std::cout<<"The center depth is "<<depth_data<<"m\n";
    img1 = cv_ptr->image;//temp mat
//    cv::imshow(OPENCV_WINDOW,img1);
//    cv::waitKey(3);

//    cv::namedWindow("Depth_subscribe",WINDOW_AUTOSIZE);
//    cv::imshow(img1);


    depth_data.clear();

    if(Crop)
    {
      cv::Rect myROI(px1, py1, px, py);// Setup a rectangle to define your region of interest (x,y,width,height)
      cv::Mat croppedImage = img1(myROI);// Crop the full image to that image contained by the rectangle myROI
      x=(croppedImage.cols);
      y=(croppedImage.rows);
      //Push back the cropped depth data
      for(int height=0; height<y; height++){ 
        float *data = croppedImage.ptr<float>(height);
        for(int width=0; width<x ; width++){
            depth_data.push_back(data[width]);
        }
      }
    }
    
    else
    {
      for(int height=0; height<y; height++){
        float *data = cv_ptr->image.ptr<float>(height);
        for(int width=0; width<x ; width++){
            depth_data.push_back(data[width]);
        }
      }
    }
     
  
  
    std::vector<float>::iterator it;
    std::ofstream dfile(filename.c_str(), std::ofstream::out | std::ofstream::trunc);//std::ios::binary | std::ios::app| 


    std::cout<<"writing depth data to csv..."<<"\n";
    std::cout<<filename<<"\n";
    int count =0;
	  for(it=depth_data.begin(); it!=depth_data.end(); ++it)
    {
       count++;
       dfile<<*it<<",";
       if(count%x== 0)
          dfile<<"\n"; 
    }
    std::cout<<"\n";
//    cv::waitKey(3);

  }





};

int main(int argc, char** argv)
{      
  ros::init(argc, argv, "depth_subscribe");
  
  std::cout<<"Please enter filename ...\n";
  std::getline(std::cin, filename);
  filename="/home/cirlab/Desktop/depth_data"+filename+".csv";
	// stringStream<<"/home/nvidia/"<<filename<<".txt";//save as avi 
	// filename = stringStream.str();
  
  ImageConverter ic;

  while(ros::ok())
  {
    ros::Rate r(1);
    ros::spinOnce();
    if(cv::waitKey(33) == 27)//press esc to escape
    { 
       break;
    }
    r.sleep();
  }
  return 0;
}
