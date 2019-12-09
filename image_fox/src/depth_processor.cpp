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

#include "std_msgs/Float32MultiArray.h"

#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <time.h>

#define Col 640 // image data width
#define Row 480 // image data height

static const std::string OPENCV_WINDOW = "Image (depth) window";
cv::Mat img_depth(Col,Row,CV_32FC1);//save the depth image
cv::Mat img_rgb(Col,Row,CV_8SC3);

std::vector<float> depth_data;
std_msgs::Float32MultiArray msg_depth;

int x,y = 0;
double block_nan_counter;
float left_depth;
float right_depth;
float align_depth;



std::ostringstream stringStream;
std::string filename;

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_depth_sub_;// subscribe depth_image from sensor
    image_transport::Publisher image_depth_pub;  // publish depth data for rgb processor
    image_transport::Subscriber image_rgb_sub_;  // subscribe rgb_image from sensor
    image_transport::Publisher image_rgb_pub;    // publish rgb image for rgb processor

public:
    ImageConverter()
        : it_(nh_)
    {
        //
        image_depth_sub_ = it_.subscribe("/camera/depth/image_rect",1,&ImageConverter::depthCallback, this);
//        image_rgb_sub_ = it_.subscribe("/camera/rgb/image_raw",1,&ImageConverter::depthCallback, this);

        cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter()
    {
      cv::destroyWindow(OPENCV_WINDOW);
    }
    void depthCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr_depth;
        try
        {
            //try to convert sensor msg to cv Mat by Cv_bridge
            //(msg,encoding_type)
            cv_ptr_depth = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        img_depth = cv_ptr_depth->image;
//        std::cout<<sizeof(cv_ptr_depth->image)<<std::endl;

        int x = cv_ptr_depth->image.cols;
        int y = cv_ptr_depth->image.rows;
//        int Total_Pixel = x*y;//You can print the Total_Pixel to see how many pixels in the depth_image
        int counter = 0;
        int Round = 0;

        depth_data.clear();
        std::vector<float>().swap(depth_data);

        for( int height = 0 ; height < Row ; height++ )
        {
            float *data =cv_ptr_depth->image.ptr<float>(height);
//            std::cout<<*data<<std::endl;
            for( int width = 0 ; width < Col ; width++ )
            {
                depth_data.push_back(data[width]);
//                counter++;
//                std::cout<<"Do time (Total) : "<<depth_data.size()<<std::endl;
            }
//            Round++;
//            std::cout<<"--------------------Do time (Round) : "<<Round<<"--------------------"<<std::endl;
        }

//        std::cout<<"--------------------"<<std::endl;
//        std::cout<<"Total element in depth_data : "<<depth_data.size()<<std::endl;
    }
    double DepthCalculation(int width_1,int width_2,int height_1, int height_2)
    {
       block_nan_counter = 0;

       int right_count = 0;
       int left_count = 0;
       int counter = 0;

       for(int y = 0 ; y < Row ; y++)
       {
           int width_temp = Col;
           for(int x = 0 ; x < Col ; x++,counter ++)
           {
               if( std::isnan(depth_data[x+y*Col]))
               {
                   if(width_1<=x && x<width_2 && height_1<=y&& y<height_2)
                   {
                        block_nan_counter ++;
                   }
//                   depth_data[x+y*Col] = 0;
               }
           }
       }
//       std::cout<<"width : "<<(width_2-width_1)<<"\t"<<"height : "<<(height_2-height_1)<<std::endl;
//       std::cout<<"Total pixel : "<<(width_2-width_1)*(height_2-height_1)<<std::endl;
//       std::cout<<"Do time (Total) : "<<counter<<std::endl;
       block_nan_counter = block_nan_counter * 100 /((width_2-width_1)*(height_2-height_1));
       return block_nan_counter;
    }
    void depth_align()
    {

        int left_count = 0;
        int right_count = 0;

        for ( int y = 0 ; y < Row ; y++ )
        {
            for ( int x = 0 ; x < Col ; x++)
            {
                if( !( std::isnan( depth_data[x+y*Col] ) ) && (213-40)<=x && x<=(213+60) && 240<=y && y<=241 )
                {
                    left_depth = left_depth + depth_data[x+y*Col];
                    left_count ++;

                }
                else if( !( std::isnan( depth_data[x+y*Col] ) ) && (426-60)<=x && x<=(426+40) && 160<=y && y<=320)
                {
                    right_depth = right_depth + depth_data[x+y*Col];
                    right_count ++;
                }
            }
        }

        if(left_count!=0)
        {
            left_depth = left_depth / left_count;
        }
        else {
            left_depth = 0;
        }
        if(right_count!=0)
        {
            right_depth = right_depth / right_count;
        }
        else {
            right_depth = 0;
        }
        align_depth = left_depth-right_depth;
    }


};

class CSVReader
{
    std::string fileName;
    std::string delimeter;
public:
    CSVReader(std::string filename,std::string delm = ","):
        fileName(filename),delimeter(delm)
    {

    }
    //Function to fetch data from a CSV File
    /*
    * Parses through csv file line by line and returns the data
    * in vector of vector of strings.
    */
    std::vector<std::vector<std::string>> getData()
    {
        std::ifstream file(fileName.c_str());
        std::vector<std::vector<std::string>> dataList;

        std::string line = "";
        // Iterate through each line and split the content using delimeter
        while(getline(file,line))
        {
            std::vector<std::string> vec;
            boost::algorithm::split(vec,line,boost::is_any_of(delimeter));
            dataList.push_back(vec);
        }
        //Close the File
        file.close();
        return dataList;
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "depth_processor");
    ros::NodeHandle nh;
    ros::Publisher depth_count_pub = nh.advertise<std_msgs::Float32MultiArray>("/depth_scan/percentage",10);

    ImageConverter ic;


    ros::Rate r(10);//determine the fps of the imshow

    int counter = 0;

    while(ros::ok())
    {

        ros::spinOnce();
        msg_depth.data.clear();
        left_depth = 0;
        right_depth = 0;
        align_depth = 0;
        //test img_depth
        /*  show the depth image to check if the msg
            is successfully recieved by image_depth_sub  */
//        ic.DepthCalculation(213,426,160,320);
        msg_depth.data.push_back( ic.DepthCalculation(213,426,160,320) );
        msg_depth.data.push_back( ic.DepthCalculation(213,426,320,480) );

        ic.depth_align();
        msg_depth.data.push_back(align_depth);


//        if(msg_depth.data[0] < 90)
//        {
//            ic.depth_align();
////            align_depth = left_depth - right_depth;
//            if( 20 < msg_depth.data[0] && msg_depth.data[0] < 50 && left_depth != 0 && right_depth != 0) //the first judge condition
//            {
//                if( msg_depth.data[1] > 60 )  //the seccond judge condition
//                {
//                    msg_depth.data.push_back( align_depth );
//                }
//                else
//                {
//                    msg_depth.data.push_back( 0 );
//                }
//            }

//        }
//        else
//        {
//            msg_depth.data.push_back( 0 );
//        }

        std::cout<<"Signal[0] : "<<(int)msg_depth.data[0]<<"\t\t"<<"Signal[1] : "<<(int)msg_depth.data[1];
        std::cout<<"\t\t"<<"Left_depth : "<<left_depth<<"\t\t"<<"Right_depth : "<<right_depth;
        std::cout<<"\t\t"<<"Align_depth : "<<align_depth;
        std::cout<<std::endl;
        cv::imshow(OPENCV_WINDOW,img_depth);
        cv::waitKey(1);//(int delay sec) 1sec is recommanded
                       //can be set by ros::Rate ,too
//        printf("%d\n",counter);

        msg_depth.data.push_back(left_depth);
        msg_depth.data.push_back(right_depth);

        depth_count_pub.publish(msg_depth);

//        counter++;


        r.sleep();
    }
    return 0;
}
