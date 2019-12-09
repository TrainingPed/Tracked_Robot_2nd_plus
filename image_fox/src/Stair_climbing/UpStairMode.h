#ifndef _UPSTAIRMODE_H_
#define _UPSTAIRMODE_H_

#include "ros/ros.h"
#include <stdlib.h>
#include <stdio.h>
#include <sstream>

#include <string>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/PointCloud.h"
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
#include <dynamixel_workbench_msgs/JointCommand.h>

#define PI 3.14
#define Resolution 2048
#define Ratio 50
class UpStairMode
{
	public:
	    std::string NameSpace;
		UpStairMode()
		{
			pub_rs = n.advertise<std_msgs::Int32MultiArray>("robot_speed", 1);
			pub_ls = n.advertise<std_msgs::Int32MultiArray>("leg_speed", 1);
			pub_rMA = n.advertise<std_msgs::Int32MultiArray>("robot_MA", 1);
			pub_lMA = n.advertise<std_msgs::Int32MultiArray>("leg_MA", 1);
			pub_rHO = n.advertise<std_msgs::Int32MultiArray>("robot_HO", 1);
			pub_rVA = n.advertise<std_msgs::Int32MultiArray>("robot_VA", 1);
			pub_rm = n.advertise<std_msgs::Int32>("robot_motion", 1);
			pub_lm = n.advertise<std_msgs::Int32>("leg_motion", 1);
			pub_lHO = n.advertise<std_msgs::Int32MultiArray>("leg_HO", 1);
			pub_lVA = n.advertise<std_msgs::Int32MultiArray>("leg_VA", 1);
			pub_rMR = n.advertise<std_msgs::Int32MultiArray>("robot_MR", 1);
			pub_MS = n.advertise<std_msgs::Int32MultiArray>("motor_switch", 1);


			
            joint_command_client = n.serviceClient<dynamixel_workbench_msgs::JointCommand>("joint_command");
			ros::param::param<std::string>("USM_NameSpace", NameSpace, "/Up_Stair_Mode");
			// pub_ht = n.advertise<std_msgs::Float64>("head_top", 1);
			// pub_hb = n.advertise<std_msgs::Float64>("head_bottom", 1);
            enc_sub = n.subscribe("Rd_enc2", 10000,&UpStairMode::ReadEnc2,this);
			sub_d21 = n.subscribe("d21", 1, &UpStairMode::d21Callback,this);
			sub_d4 = n.subscribe("d4", 1, &UpStairMode::d4Callback,this);
			sub_cnt21 = n.subscribe("cnt21", 1, &UpStairMode::cnt21Callback,this);
			sub_mid = n.subscribe("midxy", 1, &UpStairMode::midCallback,this);
            laser_sub = n.subscribe("/scan", 1,&UpStairMode::lasercallback,this);
            sub_MD21=n.subscribe("MD21",1,&UpStairMode::MD21callback,this);
            ultrasonic_sub=n.subscribe("ultrasonic",1,&UpStairMode::us_callback,this);
            UStop_sub = n.subscribe("U_Stop",100,&UpStairMode::ustop_callback,this);
            get_vo = n.subscribe("/zed/odom", 1, &UpStairMode::vodom_callback, this);
            get_depth_count = n.subscribe("/depth_scan/percentage", 1, &UpStairMode::depth_scan_callback,this);
            Encoders.assign(4,0);
        }
		void start();
		int mode_21();
        int mode_22();
        int mode_23(double vv,double Lfar_dis,double Lclose_dis,double Lstop_align_dis,bool leglock);
        void stair_alignment(double err);
        void mode_3();
		void mode_4();
        void mode_41(std::string dir,int angle);
        void mode_align();
		void legMA(int front,int back);
		void robotMA(int left,int right);
		void robotspeed(int left,int right);
		void legspeed(int front,int back);
		void robotmotion( std::string rm);
		void legmotion(std::string lm);
		void leghome(int front,int back);
		void motor_switch(int number,int serve_status);
        void vw_control(float v, float w);
        void wall_following_r(double rw_min,double rw_max,double vv,double ww,double Lfar_dis,double Lclose_dis,double Lstop_align_dis);
        void wall_following_l(double lw_min,double lw_max,double vv,double ww,double Lfar_dis,double Lclose_dis,double Lstop_align_dis);
        void wall_alignment_r(double rw_min,double rw_max);
        void wall_alignment_l(double rw_min,double rw_max);
        void wall_approach_laser(double vv,double ww,double Lfar_dis,double Lclose_dis,double Lstop_align_dis);
        double getlaser(int number);
        double BubbleSort(int array_size, double array[]);
        bool leg_angle(int number,int angle);
        void legMA_feedback(int front,int back);

		void d21Callback(const std_msgs::Int32::ConstPtr& msg_d21);
		void d4Callback(const std_msgs::Int32::ConstPtr& msg_d4);
		void cnt21Callback(const std_msgs::Int32::ConstPtr& msg_cnt21);
		void midCallback(const std_msgs::Int32MultiArray::ConstPtr& array_mid);
        void lasercallback(const sensor_msgs::LaserScan::ConstPtr& msg);
        void MD21callback(const std_msgs::Int32::ConstPtr& msg);
        void us_callback(const std_msgs::Int32::ConstPtr& msg);
        void ustop_callback(const std_msgs::Int32::ConstPtr& msg);
        void ReadEnc2(const std_msgs::Int32MultiArray::ConstPtr& array);
        void vodom_callback(const nav_msgs::Odometry::ConstPtr&msg);

        void depth_scan_callback(const std_msgs::Float32MultiArray msg_depth);
        int depth_scan_align();
        bool depth_scan_hit(int limit_0,int limit_1);
        double Align180();

		ros::NodeHandle n;
		int d21,d4;
		int counter21;
		int mid_x,mid_y;
        double laser_340=0,laser_170=0,laser_260=0,laser_2=0,laser_32=0,laser_62=0,laser_451=0,laser_481=0,laser_511=0;
        double laser_180 = 0,laser_150 =0 ;
        double laser_210 = 0,laser_225 = 0,laser_270 = 0;
        double laser_135 = 0 ,laser_90 = 0;
        double align_laser = 0 ;


        double laser[360];//for lidar
//      double laser[512];

        int MD21;
        int ultrasonic;
        int target1=0;
        int safe_switch=0;
        double vox=0,voy=0,roll=0,yaw=0,pitch=0;
        int signal[2] = {0,0};
        float align_signal;
        float left_signal,right_signal;

        std::vector <int> Encoders;
	private:
		ros::Subscriber sub_d21;
		ros::Subscriber sub_d4;
		ros::Subscriber sub_cnt21;
		ros::Subscriber sub_mid;
        ros::Subscriber laser_sub;
        ros::Subscriber sub_MD21;
        ros::Subscriber ultrasonic_sub;
        ros::Subscriber UStop_sub;
        ros::Subscriber enc_sub;
        ros::Subscriber get_vo;
        ros::Subscriber get_depth_count;

		ros::Publisher pub_rs ;
		ros::Publisher pub_ls ;
		ros::Publisher pub_rMA ;
		ros::Publisher pub_lMA ;
		ros::Publisher pub_rHO ;
		ros::Publisher pub_rVA ;
		ros::Publisher pub_MS;
		ros::Publisher pub_rm ;
		ros::Publisher pub_lm ;

		ros::Publisher pub_lHO ;
		ros::Publisher pub_lVA ;
		ros::Publisher pub_rMR ;

		// ros::Publisher pub_ht ;
		// ros::Publisher pub_hb ;
		ros::ServiceClient joint_command_client;
		int r_motion;
		int l_motion;
		int r_speed[2];
		int l_speed[2];
		int r_MA[2];
		int l_MA[2];
		int r_VA[2];
		int l_VA[2];
		int r_HO[2];
		int l_HO[2];
		int MS[2];

		

//---robot_motion---
//1=fwd
//2=back
//3=left
//4=right
//0=stop
//-------------------	 	 
//---lef_motion---
//1=front up
//2=front down 
//3=back up
//4=back down
//0=stop
//-------------------
};

#endif
