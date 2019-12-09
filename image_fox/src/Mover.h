#ifndef PI
#define PI 3.14
#endif

#ifndef _MOVER_H
#define _MOVER_H
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"
#include <vector>
#include <math.h>
#include "stdio.h"
#include <iostream>
#include <fstream>
#include <assert.h>
#include <string>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>



#define ARRAYSIZE 1500
#define PIM 0.01745// PI/180
#define tr  1000;
#define Ratio 50

using namespace std;

static int SW = 0;
static bool path_follow;
static double vr[ARRAYSIZE] = { 0.0 };
static double wr[ARRAYSIZE] = { 0.0 };
// double xr[ARRAYSIZE] = { 0.0 };
// double yr[ARRAYSIZE] = { 0.0 };
static string dir;
static vector<double> xr;
static vector<double> yr;
static double thr[ARRAYSIZE] = { 0.0 };
static double alpha1 = 3.39*0.34;
static double alpha2 = 9.0*0.1*0.5;
static double alpha3 = 23.0*0.9;
static double CLOCK_SPEED = 2;
static double UTT = 16;
static double x_current = 0.0; //used to move
static double y_current = 0.0; //used to move
static double theta_current = 0.0; //used to move
static double RRR = 2.5;// MOVING RATE
static float sonarx[8] = { 10.0 };
static float sonary[8] = { 10.0 };
static float sonardist = 1.0;
static float sonar0 = 2.0;
static float sonar7 = -2.0;
static double xot = 0;
static double yot = 0;
static double tho_t = 0;
static double xat = 0;
static double yat = 0;
static double tha_t = 0;
static double xm = 0;
static double ym = 0;
static double thm = 0;
static double th_accum = 0;
static double old_th = 0;
static double theta_track = 0;
static double old_x;
static double old_y;
static float laser340=0;
static float laser360=0;
static float laser380=0;
static float laser2 = 0;
static float laser718 = 0;
static float laser[512]={0};
static bool obstacle=1;
static bool odom_ban=0;
static string old_status;
static string old_Floor;
static vector <double> xo;
static vector <double> yo;
static vector <double> tho;
class velocity
{
	public:
		double v;
		double w;
		velocity()
		{
			v = 0;
			w = 0;
        }
	private:
};
class AMCL
{
	public:
		double x ;
		double y ;
		double th ;
        double ox;
        double oy;
        double oz;
        double ow;
        double covx;
        double covy;
        double covth;
		AMCL()
		{
			x = 0;
			y = 0;
			th = 0;
            ox = 0;
            oy = 0;
            oz = 0;
            ow = 0;
            covx=0;
            covy=0;
            covth=0;
        }

	private:
};
class odometry
{
	public:
		double x ;
		double y ;
		double th ;
        double ox;
        double oy;
        double oz;
        double ow;
		odometry()
		{
			x = 0;
			y = 0;
			th = 0;
            ox = 0;
            oy = 0;
            oz = 0;
            ow = 0;
        }

	private:
};



class robot
{
public:

    ros::NodeHandle nh;
    string filename;

	robot()
	{
        rm_pub= nh.advertise<std_msgs::Int32>("robot_motion", 1);
        rs_pub= nh.advertise<std_msgs::Int32MultiArray>("robot_speed", 1);
        motor_pub= nh.advertise<std_msgs::Int32MultiArray>("motor_switch", 1);
		cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/vw_control", 1);
		get_pose = nh.subscribe("odom", 1, &robot::odom_callback, this);
		//sonar_sub = nh.subscribe("RosAria/sonar", 1, &robot::sonarcallback, this);
		get_amcl = nh.subscribe("amcl_pose", 1, &robot::amcl_callback, this);
        resetodom_pub = nh.advertise<std_msgs::Int32>("/RosAria/resetodom", 1);
		astar_sub = nh.subscribe("path", 1, &robot::astarCallback, this);
		sw_sub = nh.subscribe("/switch", 1, &robot::SWcallback, this);
		laser_sub = nh.subscribe("/scan", 1,&robot::lasercallback,this);
		get_vo = nh.subscribe("/zed/v_odom",1,&robot::vo_callback,this);
		map_pub=nh.advertise<std_msgs::Int32>("/change_map", 1);	
        path_pub = nh.advertise<nav_msgs::Path>("trajectory",1, true);
        odom_path_pub = nh.advertise<nav_msgs::Path>("odom_trajectory",1);
        pose_pub = nh.advertise<std_msgs::Float32MultiArray>("pose_set",1);
        odom_set = nh.advertise<std_msgs::Float32MultiArray>("set_odom",1);
        rp_pub = nh.advertise<std_msgs::Float32MultiArray>("robot_pose",1);

	}
	void sonarcallback(const sensor_msgs::PointCloud::ConstPtr&);
	void amcl_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&);
	void odom_callback(const nav_msgs::Odometry::ConstPtr&);
	void astarCallback(const nav_msgs::Path& msg);
	void SWcallback(const std_msgs::Int32::ConstPtr &msg);
	void lasercallback(const sensor_msgs::LaserScan::ConstPtr& msg);
	void vo_callback(const nav_msgs::Odometry::ConstPtr& msg);
    /*void a_star_callback_x(const std_msgs::Float64MultiArray::ConstPtr&);
    void a_star_callback_y(const std_msgs::Float64MultiArray::ConstPtr&);*/

    void robotmotion(std::string rm);
    void robotspeed(int left,int right);
    void vw_control(float v, float w);
    velocity kinematic(int i, double x, double y, double x1, double y1, int amcl_modify,string path_name);
	double coordinate_transform(int, double, double, double);
	void U_turn(int, double);
	void turn_left_45(int, double);
	void angle_simplify(int i);
	void turn_left(int i);
	void turn_right(int i);
	void pause();
	void stop();
	void obstacle_avoid();
	void along_rightwall(double x_target, double y_target, double dist, double v, double w, double theta);
	void along_rightwall2(double x_target, double y_target, double dist, double v, double w, double theta);
    void fusion(int i, int amcl_modify,string path_name_x);
	void resetodom();
	void pause_time(double t);
	void make_a_move(double v, double w, double t);
	void turn_any_angle(double, int, double);
	void send_position(double x, double y, string Floor, string status);
	void change_map(int map);
	void align_elevator();
	void modify_elevator_x();
	void modify_elevator_y();
	double max(double );
	
	void Astar_path();
    void Custom_path(std::string path_name_x,std::string path_name_y,int amcl_modify);
    void Custom_linear_path(string path_name_x ,string path_name_y,int amcl_modify);
    void reset_fusion(std::string reset_mode);
    void reset_position();//reset odom and amcl
    void Pathpub(vector<double> xr,vector<double> yr);
    void OdomPathpub(double x,double y,double th);
    void savetotxt(string);
    void pathplanning(vector<double> len,vector<double>ang,std::string ,double k,double pose_x,double pose_y,double pose_th);
    void Banlist(vector<double> x,vector<double> y,vector<double> r,std::string name);
    void Posepub(double px,double py,double pz,double ox,double oy,double oz,double ow,double theta,bool od_caliberate);
    void MotorSwitch(int motor_number,int servostatus);
    bool Banrange(string path_name_x);
    void send_robotpose(double x,double y);
    ros::Time current_time, last_time;
private:

    ros::Publisher cmd_vel_pub;
    ros::Publisher resetodom_pub;
    ros::Subscriber get_pose;
    ros::Subscriber sonar_sub;
    ros::Subscriber get_amcl;
    ros::Subscriber astar_sub;
    ros::Subscriber sw_sub;
    ros::Subscriber laser_sub;
    ros::Subscriber get_vo;

    ros::Publisher map_pub;
    ros::Publisher rm_pub;
    ros::Publisher rs_pub;
    ros::Publisher path_pub;
    ros::Publisher odom_path_pub;
    ros::Publisher pose_pub;
    ros::Publisher odom_set;
    ros::Publisher motor_pub;
    ros::Publisher rp_pub;

    AMCL a;
    odometry o;
    odometry vo;
    velocity s;
    nav_msgs::Path path;
    nav_msgs::Path o_path;

    double pose[8];



	
};
#endif
