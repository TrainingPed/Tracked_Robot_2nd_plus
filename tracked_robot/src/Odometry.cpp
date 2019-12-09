#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include <iostream> 
#include <vector>
#include <math.h>
#include <algorithm>
#include <cassert>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#define PI 3.14
// #define R 0.06828 //Wheel Radius = 7.828
// #define W 0.84112 //Wheel distance = 51.72
#define Resolution 2048
#define Ratio 50
double R=0.06828;
double W=0.84112;

double Res_Mot = Resolution * Ratio;
double A_Circle = 2 * R*PI;
double LPosPrev = 0;//Left Encoder Previous Turns
double RPosPrev = 0;//Right Encoder Previous Turns
double LPos = 0;//Left Encoder Turns
double RPos = 0;//Right Encoder Turns
double SR = 0;
double SL = 0;
double SLPrev = 0;//Left Previous Distance
double SRPrev = 0;//Right Previous Distance
double dtheta = 0;
double r = 0;
double vr = 0;
double vl = 0;
double x = 0;
double y = 0;
double theta = 0;
double dt = 0;
double vx = 0;
double vy = 0;
double vth = 0;


std::vector <int> Encoders;
class Odom
{
public:
	Odom()
	{
		//Subscribe
		enc_sub = n.subscribe("Rd_enc2", 10000, &Odom::ReadEnc2,this);
		//Publish
		odom_pub = n.advertise<nav_msgs::Odometry>("odom", 100);
//        motor_switch=n.advertise<std_msgs::Int32MultiArray>("motor_switch",1);
        resetodom = n.subscribe("/RosAria/resetodom",100,&Odom::reset_odom,this);
        setodom =n.subscribe("set_odom",100,&Odom::set_odom,this);
		last_time = ros::Time::now();	//last time 
		current_time = ros::Time::now();//current time
	}
	void updateOdometry();
	void pop_front(std::vector <int> &vec, int Cap);
    void ReadEnc2(const std_msgs::Int32MultiArray::ConstPtr& array);
    void set_odom(const std_msgs::Float32MultiArray::ConstPtr& array);
    void reset_odom(const std_msgs::Int32::ConstPtr& msg);
    void PubOdom(double x, double y, double theta);
    void PubOdomwithouttf(double x, double y, double theta);
//    void MotorSwitch(int motor_number,int status);


	ros::NodeHandle n;
	ros::Subscriber enc_sub;
	ros::Publisher odom_pub;
    ros::Subscriber resetodom;
    ros::Subscriber setodom;
//    ros::Publisher motor_switch;
	ros::Time current_time, last_time;
	tf::TransformBroadcaster odom_broadcaster;

};

void Odom::pop_front(std::vector <int> &vec, int Cap)
{
	assert(vec.size() < 54);
	while (vec.size()>Cap&& ros::ok())//Check if vector size is bigger than capacity
	{
		std::iter_swap(vec.begin(), vec.begin() + 1);//free the front two element
		std::iter_swap(vec.begin()+2, vec.begin() + 3);//free the front two element
		vec.erase(vec.begin());//delete from vector
		vec.erase(vec.begin()+1);//delete from vector
		vec.erase(vec.begin()+2);//delete from vector
		vec.erase(vec.begin()+3);//delete from vector
	}

}
//void Odom::MotorSwitch(int motor_number, int status)
//{
//    std_msgs::Int32MultiArray motorswitch;
//    motorswitch.data.push_back(motor_number);
//    motorswitch.data.push_back(status);
//    motor_switch.publish(motorswitch);
//    ros::spinOnce();

//}
void Odom::reset_odom(const std_msgs::Int32::ConstPtr& msg)
{
    int reset=0;
    reset=msg->data;
    if(reset)
    {
        x=0;
        y=0;
        theta=0;
    }
}
void Odom::set_odom(const std_msgs::Float32MultiArray::ConstPtr &array)
{
//    double reset_motor=0;
    x=array->data[0];
    y=array->data[1];
    theta=array->data[2];
//    reset_motor=array->data[3];
    //    if(reset_motor == 1.0)
    //    {
//    MotorSwitch(2,0);
//    usleep(500000);
//    MotorSwitch(3,0);
//    usleep(500000);
//    MotorSwitch(2,1);
//    usleep(500000);
//    MotorSwitch(3,1);
//    usleep(500000);
//    printf("reset motor ...");
//    }
}
void Odom::ReadEnc2(const std_msgs::Int32MultiArray::ConstPtr& array)
{
	
	for (int j = 0; j < 4; j++)
	{
		Encoders.push_back(*(array->data.begin() + j));
		/*	std::cout << "Encoder " << j << " is " << Encoders.at(j) << std::endl*/;
		pop_front(Encoders, 52);
	}
	current_time= ros::Time::now();
	dt = (current_time - last_time).toSec();//sampling time 
	last_time = ros::Time::now();
	std::cout << "Encoder 1: " << *(Encoders.end() - 4) << " Encoder 2: " << *(Encoders.end() - 3) << std::endl;
	std::cout << "Encoder 3: " << *(Encoders.end() - 2) << " Encoder 4: " << *(Encoders.end() - 1) << std::endl;
	
	updateOdometry();//Update the Odemetry
}

void Odom::updateOdometry()
{
	if( !n.getParam("odom_R", R) )
    ROS_ERROR("Failed to get odom_R from server.");
	if( !n.getParam("odom_W", W) )
    ROS_ERROR("Failed to get odom_W from server.");
	
	ros::spinOnce();               // check for incoming messages
	
	LPos = -*(Encoders.end() - 3) / Res_Mot;//Left Tracked Motor
	RPos = *(Encoders.end() - 2) / Res_Mot;//Right Tracked Motor

	SL = A_Circle * (LPos - LPosPrev);//Left Distance
	SR = A_Circle * (RPos - RPosPrev);//Right Distance 

	LPosPrev = LPos; //Previous LPos
	RPosPrev = RPos; //Previous RPos


	dtheta = (SR - SL) / W;
	theta += dtheta; //theta update

	if (theta > 2 * PI)//theta > 2PI
		theta -= 2 * PI;
	else if (theta < -2 * PI)//theta > -2PI
		theta += 2 * PI;

	double SMean = (SL + SR) / 2;//Mean Distance
	
	

	
	
	r = SMean; // /dtheta;//r * dtheta = SMean

	//if ((dtheta < 0.01) && dtheta > -0.01)
	//{
	//	r = SMean;
	//}

	x = x + r * cos(theta);
	y = y + r * sin(theta);

	vl = (SL)/dt;//v = dl/st
	vr = (SR)/dt;
	
	vx = r * cos(theta)/dt;
	vy = r * sin(theta)/dt;
	vth = dtheta/dt;



	SLPrev = SL;
	SRPrev = SR;

    Odom::PubOdom(x,y,theta);//Publish

	std::cout << "vL: " << vl << " vR: " << vr << std::endl;
	std::cout << "vx: " << r * cos(theta)/dt << " vy: " << r * sin(theta)/dt << std::endl;
	std::cout << "LPos: " << LPos << " RPos: " << RPos << std::endl;
	std::cout << "SL: " << SL << " SR: " << SR << " SMean: " << SMean << std::endl;
	std::cout << "x: " << x << " y: " << y << " theta: " << theta/(PI/180) << std::endl<< std::endl;
	
}
void Odom::PubOdom(double x, double y, double theta)
{
	//construct a Quaternion Message from yaw only (about the Z axis)

	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

	//first, we'll publish the transform over tf
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

	odom_trans.transform.translation.x = x;
	odom_trans.transform.translation.y = y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;

	//send the transform
	odom_broadcaster.sendTransform(odom_trans);

	//next, we'll publish the odometry message over ROS

	nav_msgs::Odometry odom;// ob'_'ov
	odom.header.stamp = current_time;
	odom.header.frame_id = "odom";

	//Position
	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;

	//Velocity
	odom.child_frame_id = "base_link";
	odom.twist.twist.linear.x = vx;
	odom.twist.twist.linear.y = vy;
	odom.twist.twist.angular.z = vth;


	odom_pub.publish(odom);//Publish odom message
	printf("Odometry has published ...\n");
	ros::spinOnce();
}
void Odom::PubOdomwithouttf(double x, double y, double theta)
{
    //construct a Quaternion Message from yaw only (about the Z axis)

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

    //next, we'll publish the odometry message over ROS

    nav_msgs::Odometry odom;// ob'_'ov
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //Position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //Velocity
//	odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;


    odom_pub.publish(odom);//Publish odom message
    printf("Odometry has published ...\n");
    ros::spinOnce();
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "Odometry");

	Odom odom;
	odom.n;

	Encoders.reserve(50);
	for (int j = 0; j < 4; j++)
	{
		Encoders.push_back(0);
	}
	while (ros::ok())
	{
		ros::spin();
	}

	return 0;
}


