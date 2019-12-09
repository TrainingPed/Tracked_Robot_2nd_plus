#ifndef _STAIRSDET_H_
#define _STAIRSDET_H_
#include "ros/ros.h"

#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"
//odom
#include "geometry_msgs/Point32.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>


#define row 672
#define col 376

static const std::string OPENCV_WINDOW = "RGB window";
static const std::string OPENCV_WINDOW2 = "Depth window";


class StairDetection
{
	public:
		std::string NameSpace;
		StairDetection():it(n)
		{
            d_sub = it.subscribe("/zed/depth/depth_registered", 1, &StairDetection::imageCallback,this);
            rgb_sub=it.subscribe("/zed/rgb/image_rect_color", 1, &StairDetection::rgbCallback,this);
            get_vo = n.subscribe("/zed/odom", 1, &StairDetection::vodom_callback, this);
            pub_d21 = n.advertise<std_msgs::Int32>("d21", 1);
			pub_cnt21 = n.advertise<std_msgs::Int32>("cnt21", 1);
			pub_d4 = n.advertise<std_msgs::Int32>("d4", 1);
			pub_mid = n.advertise<std_msgs::Int32MultiArray>("midxy", 1);
            pub_mid21 = n.advertise<std_msgs::Int32MultiArray>("midxy21", 1);
            pub_21MD = n.advertise<std_msgs::Int32>("MD21", 1);

			pub_d5 = n.advertise<std_msgs::Int32>("d5", 1);
			pub_d7 = n.advertise<std_msgs::Int32>("d7", 1);
			pub_d = n.advertise<std_msgs::Int32>("d", 1);
			pub_mid5 = n.advertise<std_msgs::Int32MultiArray>("mid5", 1);
			pub_mid7 = n.advertise<std_msgs::Int32MultiArray>("mid7", 1);
			ros::param::param<std::string>("StairDet_NameSpace", NameSpace, "/Stair_Det");
		}
		void imageCallback(const sensor_msgs::ImageConstPtr& msg);
		void rgbCallback(const sensor_msgs::ImageConstPtr& msg);
		
		void test();
		void Depth_Arr();
		int depthMD(int x,int y);
        void edge_find(cv::Mat &input,int *midx,int *midy,int *di,int *dj,int *dx,int *dsw);
		void depth_find(int dc,int *mid_x2,int *mid_y2,int *midx,int *midy,int *di,int *dj,int *dx,int *dsw);
		void depth_find_4(int dc,int *midx,int *midy,int *di,int *dj,int *dx,int *dsw);
		void depth_find_5(int dc,int *midx,int *midy,int *di,int *dj,int *dx,int *dsw);
        void depth_find_7(int dc,int *midx,int *midy,int *di,int *dj,int *dx,int *dsw);
        void vodom_callback(const nav_msgs::Odometry::ConstPtr&msg);
        void SD_Start();
        int BubbleSort(int array_size,int *array);
		ros::NodeHandle n;
        double vox=0,voy=0,roll=0,yaw=0,pitch=0;
		
	private:
		
		image_transport::ImageTransport it;
		image_transport::Subscriber d_sub;
		image_transport::Subscriber rgb_sub;
        ros::Subscriber get_vo;

		ros::Publisher pub_d21;
		ros::Publisher pub_d4;
		ros::Publisher pub_cnt21;
		ros::Publisher pub_mid;
        ros::Publisher pub_mid21;
        ros::Publisher pub_21MD;

		ros::Publisher pub_d5;
		ros::Publisher pub_d7;
		ros::Publisher pub_d;
		ros::Publisher pub_mid5;
		ros::Publisher pub_mid7;
		
		float Arr[row*col];
		int depth[row*col];

};



#endif







