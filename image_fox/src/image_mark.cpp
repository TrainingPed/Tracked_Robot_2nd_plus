#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "stdio.h"
#include <vector>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#define Col 640//642(640)       can be devided by 3
//#define Col 672 //672
#define Row 480//342(340)       can be devided by 3
//#define Row 376 //376


static const std::string OPENCV_WINDOW = "Image window";

cv::Mat img1(Col,Row,CV_8UC3);//save the rgb image
cv::Mat img2(Col,Row,CV_8UC3);//image calc obstacle points
cv::Mat img_depth(Col,Row,CV_32FC1);
cv::Mat croppedImage(213,160,CV_8UC3);//save the crooped image
//cv::Mat croppedImage_3(213,160,CV_8UC3);

bool img2_finsh=0;
int blocks[3][3]={0};

std::ostringstream stringStream;
std::string filename,NameSpace;
std::vector <float> avg_depth;

std::vector <float> depth_data1;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber depth_sub_;
//   image_transport::Publisher image_pub_;
  
  public:

  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1,&ImageConverter::imageCb, this);
    depth_sub_ = it_.subscribe("/camera/depth/image_rect", 1,&ImageConverter::depthCb, this);
    ros::param::param<std::string>("image_mark_NameSpace", NameSpace, "/image_mark");
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
//      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
   
    // Draw 9 rectangle represent the range of blocks

    //top left rectangle coordinate (px1,py1) (px1+px,py1) (px1,py1+py) (px1+px,py1+py)



    int px11=213;
    int px1=px11,py1=160;
    float t1=0.2;
    int t2_o=8000;
    int t2_c=500;
    int px=213;
    int py=160;
    int px2=px1+px, py2=py1+py;
    ros::param::get(NameSpace+"/px1", px11);
     ros::param::get(NameSpace+"/px1",px11);
     ros::param::get(NameSpace+"/py1",py1);
     ros::param::get(NameSpace+"/px",px);
     ros::param::get(NameSpace+"/py",py);
     ros::param::get(NameSpace+"/t1",t1);
     ros::param::get(NameSpace+"/t2_c",t2_c);
     ros::param::get(NameSpace+"/t2_o",t2_o);
    px1=px11;

    //Create 9 blocks 
    //Create 1 block (center_block)
    for (int i=0;i<1;i++)
    {
      for (int j=0;j<1;j++)
       {
          cv::rectangle(cv_ptr->image, cv::Point(px1,py1),cv::Point(px1+px,py1+py),  cv::Scalar(0,0,255),  2,  8,  0);
//          px1=px1+px;
       }
//      py1=py1+py;
//      px1=px11;
    }
//    px1=px11+px;
//    py1=40+py/2;
    
/*    for (int j=0;j<3;j++)
    {
      cv::line(cv_ptr->image, cv::Point(px1,py1),cv::Point(px1+px,py1),  cv::Scalar(0,255,255),  2,  8,  0);
      py1=py1+py;
    }
*/
    img1 = cv_ptr->image;//temp mat
    cv_ptr->image.copyTo(img2);//temp mat
//    cv_ptr->image.copyTo(img3);
    // cv::waitKey(3);
   


  }
  void depthCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr2;
   
    try
    {
      cv_ptr2 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);//for depth_image
//      cv_ptr2 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);//for rgb_image
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  
    
    // Draw an rectangle represent the range of blocks
    //cv::Point points[3][4];
    int px11=213;
    int px1=px11,py1=160;
    float t1=0.1;
    int t2=120;
    int px=213;
    int py=160;
    int px2=px1+px, py2=py1+py;
    ros::param::get(NameSpace+"/px1",px11);
    ros::param::get(NameSpace+"/py1",py1);
    ros::param::get(NameSpace+"/px",px);
    ros::param::get(NameSpace+"/py",py);
    ros::param::get(NameSpace+"/t1",t1);
    ros::param::get(NameSpace+"/t2",t2);
    px1=px11; 

    int x=cv_ptr2->image.cols;
    int y=cv_ptr2->image.rows;

    //cv::Mat tempImg =  cv_ptr2->image;
    
    //std::cout<<"ok456??\n";
     
    // online calc average row depth 

    //  for(int height=0; height<y; height++){
    //   float *data = cv_ptr2->image.ptr<float>(height);
    //   avg_depth[height]=0;
    //   for(int width=0; width<x ; width++){
    //         if( isnan(data[width]))//process nan problem
    //             data[width]=0;
    //         else if(data[width]>20)
    //             data[width]=20; //process infinite problem
    //         else if(data[width]<0)
    //            data[width]=0;//process negative problem
    //         avg_depth[height]=avg_depth[height]+data[width];
    //       // std::cout<<depth_data[height*width]<<"\n";  
    //   }
    //   avg_depth[height]=avg_depth[height]/Col;
    // }
   img_depth = cv_ptr2->image;
   
    for(int height=0; height<y; height++){
      float *data = cv_ptr2->image.ptr<float>(height);
      cv::Vec3b *rgb = img1.ptr<cv::Vec3b>(height);//img1
//      cv::Vec3b *calc = img3.ptr<cv::Vec3b>(height);//img2
      cv::Vec3b *calc = img2.ptr<cv::Vec3b>(height);//img2
      for(int width=0; width<x ; width++){
          // if ( ((avg_depth[height]-data[width])>=t1) && (data[width]>0) ) //mark obstacle with blue points
          //   {
          //       rgb[width][0]=255;bash -c "source /opt/ros/kinetic/setup.bash && rosrun qttrack qttrack"

          //       rgb[width][1]=0;
          //       rgb[width][2]=0;
          //       calc[width][0]=1;
          //       calc[width][1]=0;
          //       calc[width][2]=0;
          //   }
          // else
          // {
          //       calc[width][0]=0;
          //       calc[width][1]=0;
          //       calc[width][2]=0;
          // }
          if(!  std::isnan(data[width]))
          {
//              calc[width][0]=0;
//              calc[width][1]=0;
//              calc[width][2]=0;
              rgb[width][0]=0;
              rgb[width][1]=255;
              rgb[width][2]=0;
          }
//          else if (data[width]==0)
//          {
//              rgb[width][0]=255;
//              rgb[width][1]=0;
//              rgb[width][2]=0;
//          }
//          else {
//              calc[width][0]=1;
//              calc[width][1]=1;
//              calc[width][2]=1;
//          }
         
      }
    }
    img2_finsh=1;

//    cv::imshow(OPENCV_WINDOW, cv_ptr2->image);
    // Update GUI Window
    
//     cv::waitKey(30);
  }

  void CropBlocks()//Crop original image into 9 blocks and calc obstacle points number.
  {
    int px11=213;
    int px1=px11,py1=160;
//    float t1=0.2;
    int t2_o = 95;//95% are NAN
//    int t2_o=8000;
    int t2_c = 95;//95% are NAN
//    int t2_c=500;
    int px=213;
    int py=160;
    int px2=px1+px, py2=py1+py;
    ros::param::get(NameSpace+"/px1",px11);
    ros::param::get(NameSpace+"/py1",py1);
    ros::param::get(NameSpace+"/px",px);
    ros::param::get(NameSpace+"/py",py);
//    ros::param::get(NameSpace+"/t1",t1);

    ros::param::get (NameSpace+"/t2_c",t2_c);;//center blocks' threshold
//    ros::param::get(NameSpace+"/t2_c",t2_c);//center blocks' threshold
    ros::param::get(NameSpace+"/t2_o",t2_o);//bottom_mid blocks' threshold
//    ros::param::get(NameSpace+"/t2_o",t2_o);//other blocks' threshold
    px1=px11;
    
    for (int i=0;i<1;i++)
    {
        for (int j=0;j<1;j++)
        {

          cv::Rect myROI(px1, py1, px, py);// Setup a rectangle to define your region of interest (x,y,width,height)
//          cv::Rect myROI_3(213, 160, 1, 1);
          croppedImage = img2(myROI);// Crop the full image to that image contained by the rectangle myROI
//          croppedImage_3 = img3(myROI_3);
//          printf("%d\n",sizeof(croppedImage)/sizeof());
//          double s=0;
//          s = cv::sum( croppedImage_3 )[0];
//          s = s/213/160*100;

//          std::cout<<"There're "<<s<<" % NAN in the blocks!"<<"\n";

//          std::cout<<"("<<1<<","<<1<<"): "<<s<<"\n";
//          if ((j==1) && (s>=t2_c))
//            blocks[i][j]=1;
//          else if (s>=t2_o)
//            blocks[i][j]=1;
//          else
//            blocks[i][j]=0;
//           cv::imshow(OPENCV_WINDOW,croppedImage);
//          px1=px1+px;
        }  
//        py1=py1+py;
//        px1=px11;
    }
  }

};

int Nan_Counter(int whidth_1,int whidth_2,int height_1,int height_2)
//define 1:start 2:end
{
    std::vector<float> depth_data = depth_data1;
    double block_nan_counter = 0;
    for(int y=0; y<Row; y++)
    {
//        avg_depth[y]=0;
        int width_temp = Col;
//      for(int x=0; x<=(Col) ; x++){//x=0 x<Col x++
        for(int x=0; x<Col ; x++)//x=0 x<Col x++
        {
//            printf("%d\n",block_nan_counter);
            if( std::isnan(depth_data[x+y*Col]))//process nan problem
            {
                depth_data[x+y*Col]=0;
                width_temp--;
                if( (whidth_1<=x)&&(x<whidth_2)&&(height_1<=y)&&(y<height_2) )       //[213][160]~[426][320]
                {
//                    printf("%d\n",block_nan_counter);
                    block_nan_counter++;
                }
            }
            else if(depth_data[x+y*Col]>20)
            {
                depth_data[x+y*Col]=20; //process infinite problem
                width_temp--;
            }
            else if(depth_data[x+y*Col]<0)
            {
                depth_data[x+y*Col]=0;//process negative problem
                width_temp--;
            }

//            avg_depth[y]=avg_depth[y]+depth_data1[x+y*Col];
        }
//        avg_depth[y]=avg_depth[y]/width_temp ;
//        std::cout<<"average depth "<<y<<":"<<avg_depth[y]<<"\n";
    }
    depth_data.clear();
    block_nan_counter = block_nan_counter*100/213/160;
    return (int)block_nan_counter;
}


class CSVReader
{
	std::string fileName;
	std::string delimeter;

public:
	CSVReader(std::string filename, std::string delm = ",") :
			fileName(filename), delimeter(delm)
	{ }

	// Function to fetch data from a CSV File
	std::vector<std::vector<std::string> > getData();
};

/*
* Parses through csv file line by line and returns the data
* in vector of vector of strings.
*/
std::vector<std::vector<std::string> > CSVReader::getData()
{
  
	std::ifstream file(fileName.c_str());

	std::vector<std::vector<std::string> > dataList;

	std::string line = "";
	// Iterate through each line and split the content using delimeter
	while (getline(file, line))
	{
		std::vector<std::string> vec;
		boost::algorithm::split(vec, line, boost::is_any_of(delimeter));
        dataList.push_back(vec);
    }
	// Close the File
	file.close();

	return dataList;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  
  // std::cout<<"Please enter filename ...\n";
  // std::getline(std::cin, filename);
	// stringStream<<"/home/nvidia/Videos/"<<filename<<".avi";//save as avi 
	// filename = stringStream.str();
  
   ImageConverter ic;

  //  std::cout<<"Please sss filename ...\n";
	// Get the no obstacle depth data from CSV File
    // CSVReader reader("/home/nvidia/Desktop/depth_data/depth_data0812.csv");
//    CSVReader reader("/home/cirlab/Desktop/depth_data/depth_data1.csv");
//    CSVReader reader("/home/cirlab/Desktop/depth_data1.csv");

//    std::vector<std::vector<std::string>> dataList = reader.getData();
//    for(int i=0;i<dataList.size();i++)
//    {
//      std::vector<std::string> vec = dataList[i];

//      for (int j=0;j<vec.size();j++)
//        depth_data1.push_back( std::atof(vec[j].c_str()) );//convert into float and save as one dimension vector
////        printf("%f\n",depth_data1[1]);
//    }

//    avg_depth.resize(Row);
//    int block_nan_counter = 0;
  //create average depth vector 
//    for(int y=0; y<Row; y++){
//      avg_depth[y]=0;
//      int width_temp = Col;
//      int nan_counter = 0;
////      for(int x=0; x<=(Col) ; x++){//x=0 x<Col x++
//      for(int x=0; x<Col ; x++){//x=0 x<Col x++
//            if( std::isnan(depth_data1[x+y*Col]))//process nan problem
//            {
//                depth_data1[x+y*Col]=0;
//                width_temp--;
//                if( (213<=x)&&(x<426)&&(160<=y)&&(y<320) )       //[213][160]~[426][320]
//                {
//                    block_nan_counter++;
//                }
//            }
//            else if(depth_data1[x+y*Col]>20)
//            {
//                depth_data1[x+y*Col]=20; //process infinite problem
//                width_temp--;
//            }
//            else if(depth_data1[x+y*Col]<0)
//            {
//                depth_data1[x+y*Col]=0;//process negative problem
//                width_temp--;
//            }

//            avg_depth[y]=avg_depth[y]+depth_data1[x+y*Col];
//        }
//      avg_depth[y]=avg_depth[y]/width_temp ;
//      std::cout<<"average depth "<<y<<":"<<avg_depth[y]<<"\n";
//    }
//    std::cout<<"There are "<<Nan_Counter<<"% Nan in the block\n";
//  std::cout<<"Pleasddde entddder filename ...\n";
  while(ros::ok())
  {
    CSVReader reader("/home/cirlab/Desktop/depth_data1.csv");


    std::vector<std::vector<std::string>> dataList = reader.getData();
//    for(int i=0;i<dataList.size();i++)
//    {
//    std::vector<std::string> vec = dataList[i];

//        for (int j=0;j<vec.size();j++)
//        {
//            depth_data1.push_back( std::atof(vec[j].c_str()) );//convert into float and save as one dimension vector
//  //        printf("%f\n",depth_data1[1]);
//        }

//    }

    img2_finsh=0;
    ros::spinOnce();
    std::cout<<"There are "<<Nan_Counter(0,640,0,320)<<"% Nan in the block\n";
 
//    if(cv::waitKey(33) == 27)//press esc to escape
//    {
//       break;
//    }
    
    if(img2_finsh)
    {  
      cv::imshow(OPENCV_WINDOW,img1);//show result
      cv::waitKey(1);
      ic.CropBlocks();
    }
//    for (int i=0;i<3;i++)
//    {
//        for (int j=1;j<3;j++)
//          std::cout<<blocks[i][j]<<",";
//        std::cout<<"\n";
//    }
  }

  return 0;
}
