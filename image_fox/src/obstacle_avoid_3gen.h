#ifndef _OBSTACLE_AVOID_3_GEN_H_
#define _OBSTACLE_AVOID_3_GEN_H_

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include <vector>
#include <math.h>
#include <iostream>
#include <fstream>
#include <assert.h>
#include <string>
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "stdio.h"
#include <iterator>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <errno.h>
#include <pthread.h>


#define Ratio 50
#define PI 3.14


#define Col 672
#define Row 376

using namespace std;

static const std::string OPENCV_WINDOW = "Image window";
static const std::string OPENCV_WINDOW2 = "Depth Image";


static int Frame=0;
static int x,y=0;
static string filename;
// vector <float> depth_data;
static double L=0.56;//0.84112 meter
static double Vel_c=0.1;//0.1m/s
static int act_old=1;//old action
static bool drive_hard=0;


static bool img2_finsh=0;
static int Blocks[3][3]={0};
static std::vector <float> avg_depth;
static std::vector <float> no_obs_depth;
static std::vector <float> depth_data;

static int rs[2];//robot_speed
static int ls[2];//leg_speed
static int rMA[2];//robot_MA
static int lMA[2];//leg_MA
static int rVA[2];//robot_VA
static int lVA[2];//leg_VA
static int rHO[2];//robot_HO
static int lHO[2];//leg_HO
static int rMR[2];//robot_MR
static int rm = 0;//robot_motion
static int lm = 0;//leg_motion
static int oldM2=0;
static int oldM3=0;


class twowheel 
{
public:
	double right;
	double left;
	twowheel(){
	double right=0;
	double left=0;
    }
};
class Distance 
{
public:
	double d1;
	double d2;
	Distance(){
    double d1=0;
	double d2=0;
    };
};

class position 
{
public:
	int x;
	int y;
	position(){
    int x=0;
	int y=0;
    };
};

static twowheel speed;
static position p;
static Distance dis;

class obstacle_avoid
{
    public:
	ros::NodeHandle nh;
	ros::Publisher  RS_pub;
    ros::Publisher  RM_pub;

    double R = 0.06828;
    double W = 0.73;
    double theta = 30;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Subscriber depth_sub_;

    string NameSpace;
    obstacle_avoid(): it_(nh)
	{   
        RM_pub = nh.advertise<std_msgs::Int32>("robot_motion", 100);
		RS_pub = nh.advertise<std_msgs::Int32MultiArray>("robot_speed", 100);
        // Subscrive to input video feed and publish output video feed
        // image_sub_ = it_.subscribe("/zed/depth/depth_registered", 1,&obstacle_avoid::imageCallback, this);
        image_sub_ = it_.subscribe("/zed/rgb/image_rect_color", 1,&obstacle_avoid::imageCb, this);
        depth_sub_ = it_.subscribe("/zed/depth/depth_registered", 1,&obstacle_avoid::depthCb, this);
        
        ros::param::param<std::string>("Obstacle_NameSpace", NameSpace, "/ob3tacle_avoid");
        //  cv::namedWindow(OPENCV_WINDOW);
        //  cv::namedWindow(OPENCV_WINDOW2);
    }
    ~obstacle_avoid()
	{   
        
        // cv::destroyWindow(OPENCV_WINDOW);
        // cv::namedWindow(OPENCV_WINDOW2);
    }
    cv::Mat img1;//save the rgb image
    cv::Mat img2;//image calc obstacle points
    cv::Mat img3;//depth image 


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
    
        // Draw 9 rectangle represent the range of blocks
        int px11=15;
        int px1=px11,py1=40;
        float t1=0.01;
        int t2=120;
        int px=215;
        int py=50;
        int px2=px1+px, py2=py1+py;
        bool depth_imgshow=1;
        ros::param::get(NameSpace+"/px1", px11);
        ros::param::get(NameSpace+"/py1", py1);
        ros::param::get(NameSpace+"/px", px);
        ros::param::get(NameSpace+"/py", py);
        ros::param::get(NameSpace+"/t1", t1);
        ros::param::get(NameSpace+"/t2", t2);
        ros::param::get(NameSpace+"/depth_imgshow", depth_imgshow);
        px1=px11;
        // nh.getParam("px1",px11);
        // nh.getParam("py1",py1);
        // nh.getParam("px",px);
        // nh.getParam("py",py);
        // nh.getParam("t1",t1);
        // nh.getParam("t2",t2);
        // nh.getParam("depth_img

        //Create 9 blocks 
        for (int i=0;i<3;i++)
        {
            for (int j=0;j<3;j++)
                { 
                cv::rectangle(cv_ptr->image, cv::Point(px1,py1),cv::Point(px1+px,py1+py),  cv::Scalar(0,0,255),  2,  8,  0);
                px1=px1+px;
                }
            py1=py1+py;
            px1=px11;
        }
  
 
    img1 = cv_ptr->image;//temp mat
    //cv::imshow(OPENCV_WINDOW,img1);
    // cv_ptr->image.copyTo(img2);//temp mat
    // cv::waitKey(3);

  }
    void depthCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr2;
    
        try
        {
             cv_ptr2 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        }
        catch (cv_bridge::Exception& e)
        {
             ROS_ERROR("cv_bridge exception: %s", e.what());

        return;
        }
    
        
        // Draw an rectangle represent the range of blocks
        int px11=15;
        int px1=px11,py1=40;
        float t1=0.01;
        int t2=120;
        int px=215;
        int py=50;
        int px2=px1+px, py2=py1+py;
        bool depth_imgshow=1;
        ros::param::get(NameSpace+"px1", px11);
        ros::param::get(NameSpace+"py1", py1);
        ros::param::get(NameSpace+"px", px);
        ros::param::get(NameSpace+"py", py);
        ros::param::get(NameSpace+"t1", t1);
        ros::param::get(NameSpace+"t2", t2);
        ros::param::get(NameSpace+"depth_imgshow", depth_imgshow);
        // nh.getParam("px1",px11);
        // nh.getParam("py1",py1);
        // nh.getParam("px",px);
        // nh.getParam("py",py);
        // nh.getParam("t1",t1);
        // nh.getParam("t2",t2);
        // nh.getParam("depth_imgshow",depth_imgshow);
        px1=px11;

        int x=cv_ptr2->image.cols;
        int y=cv_ptr2->image.rows;
        img3=cv_ptr2->image;//temp mat
        cv_ptr2->image.copyTo(img2);//temp mat
        depth_data.clear();

        for(int Height=0; Height<y; Height++)
        {
            float *data = cv_ptr2->image.ptr<float>(Height);
            cv::Vec3b *rgb = img1.ptr<cv::Vec3b>(Height);//img1
            float *calc = img2.ptr<float>(Height);//img2 
            for(int Width=0; Width<x ; Width++){
                depth_data.push_back(data[Width]);
                if ( ((avg_depth[Height]-data[Width])>=t1) && (data[Width]>0) ) //mark obstacle with blue points
                    {
                        rgb[Width][0]=255;
                        rgb[Width][1]=0;
                        rgb[Width][2]=0;
                        calc[Width]=1;
                    }
                else
                {
                        calc[Width]=0;
                }

                
            }
        }
        if(depth_imgshow && img3.cols!=0)
            cv::imshow(OPENCV_WINDOW2,img3);
        // cv::imshow(OPENCV_WINDOW,img1);
        if((x!=0)&&(y!=0))
            img2_finsh=1;
        
    //    cv::normalize( img3, img3, 0.0, 1.0, cv::NORM_MINMAX);
        
        //cv::waitKey(0);

    }

    //Crop image into 9 blocks
    void CropBlocks();

    //Control the speed with radius
    void vel_controller(double r ,double Vel_c); 

    //publish speed to Motor Controller
    void vel_publisher(twowheel speed);
    
    //Transfer Blocks result to action
    void action_launcher();
   
    //Action
    void action(int act, int od,int px,int py);
    
    //Transfer depth map to actual distance
    Distance Truthdepth(int px1,int py1,int px,int py,int center);
    
    //Table of d1 d2
    Distance DepthTable(int Grid,int Num);

    //adjust the radius
    float radius_adjust(Distance dis);  

    // robot motion publish
    void rm_publish(int rm);

    //detect obstacle distribution
    int obstacle_detect();

    // obstacle_mode switch ( thread 1 )
    // void ob_switch()
    // {
    //     while(ros::ok()){
    //         y = getch();
    //         if (y == 97 || y == 65)//A for go straight
    //         {
    //             obstacle_avoid::rm_publish(1);
    //         }
    //         else if( y==90 || y==122 )//Z for stop
    //         {
    //             obstacle_avoid::rm_publish(0);
    //         }
    //         else if( y==101 || y==69 )//Z for stop
    //         {
    //             obstacle_avoid::rm_publish(0);
    //         }
    //     }
    // } 

    //This is the main thread  ( thread 0 )
    void main_work()
    {
        bool rgb_show=1;
        ros::param::get(NameSpace+"rgb_show", rgb_show);
        ros::param::get(NameSpace+"Vel_c", Vel_c);
        // nh.getParam("rgb_show",rgb_show);
        // nh.getParam("Vel_c",Vel_c);
        obstacle_avoid::rm_publish(1);
        while(ros::ok())
        { 
            ros::spinOnce();
            if(cv::waitKey(33) == 27)//press esc to escape
            { 
                break;
            }
            if (rgb_show && img1.cols !=0 )
                cv::imshow(OPENCV_WINDOW,img1);
            //show result
   
            if(img2_finsh)
            {
                obstacle_avoid::action_launcher();
            }
                
            cout<<"\n";  
            //usleep(100000);//sleep 0.1 sec
        }

    }
    // void thread_init()
    // {
    //     int NUM_THREADS=2;
    //     int rc;
    //     int i=0;
    //     pthread_t threads[NUM_THREADS];
    //     pthread_attr_t attr;
        
    //     // Initialize and set thread joinable
    //     pthread_attr_init(&attr);
    //     pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
        
    //     // pthread_create(&tid, &attr, THREAD_FUNCTION, arg);
    //     cout <<"creating  main thread, "  << endl;
    //     rc = pthread_create(&threads[i], &attr, thread1, this);
    //     if (rc) {
    //         cout << "Error:unable to create thread," << rc << endl;
    //         exit(-1);
    //     }

    //     cout <<"creating 2nd thread: obs_switch, "  << endl;
    //     i++;
    //     rc = pthread_create(&threads[i], &attr, thread2, this);
    //     if (rc) {
    //         cout << "Error:unable to create thread," << rc << endl;
    //         exit(-1);
    //     }
    //     pthread_attr_destroy(&attr);
    //     int* status = 0;
    //     pthread_join(threads[0], (void**) &status);
    //     pthread_join(threads[1], (void**) &status);

    // }
private:
// 	 static void* thread1(void* arg) {
//       obstacle_avoid* t = reinterpret_cast<obstacle_avoid*>(arg);
//       t->main_work();
//       pthread_exit(NULL);
//    }
//    static void* thread2(void* arg) {
//       obstacle_avoid* t1 = reinterpret_cast<obstacle_avoid*>(arg);
//       t1->ob_switch();
//       pthread_exit(NULL);
//    }
};
class CSVReader
{
	std::string fileName;
	std::string delimeter;

public:
	CSVReader(std::string filename, std::string delm = ",") :fileName(filename), delimeter(delm){}


    // Function to fetch data from a CSV File
    
    /*
    * Parses through csv file line by line and returns the data
    * in vector of vector of strings.
    */

    vector<std::vector<std::string> > getData()
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

};
#endif