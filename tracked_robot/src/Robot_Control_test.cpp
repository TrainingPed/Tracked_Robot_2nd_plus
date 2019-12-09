#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"
#include "geometry_msgs/Twist.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include "std_msgs/String.h"
#include <iostream> 
#include <vector>
#include <math.h>
#include <algorithm>
#include <cassert>

#define PI 3.14
// #define R 0.06828 //Wheel Radius = 7.828
// #define W 0.84112 //Wheel distance = 51.72
#define Resolution 2048
#define Ratio 50

double R=0.06828;
double W=0.84112;
class twowheel 
{
public:
	twowheel();
	double right;
	double left;
};

class velocity
{
public:
	velocity();
	double v;
	double w;
private:
};
velocity::velocity()
{
	//v and w(cm/s)
	double v = 0;
	double w = 0;
}
twowheel::twowheel()
{
	//vright vleft
	double right = 0;
	double left = 0;
}

velocity s;
velocity s_old;
twowheel speed;

double A_Circle = 2 * R*PI;
double r = 0;



//Motor (rpm)
int M2 = 0;//Motor 2
int M3 = 0;//Motor 3

double x = 0;
double y = 0;
double theta = 0;



//Publish Array

int rs[2];//robot_speed
int ls[2];//leg_speed
int rMA[2];//robot_MA
int lMA[2];//leg_MA
int rVA[2];//robot_VA
int lVA[2];//leg_VA
int rHO[2];//robot_HO
int lHO[2];//leg_HO
int rMR[2];//robot_MR
int rm = 0;//robot_motion
int lm = 0;//leg_motion
int ctr=0;//counter

class Troller
{
	public:
		Troller()
		{
			//Subscribe
			vw_sub = n.subscribe("/vw_control", 100,&Troller::vw_control, this);

			//Publish
			RS_pub = n.advertise<std_msgs::Int32MultiArray>("robot_speed", 100);
			RM_pub = n.advertise<std_msgs::Int32>("robot_motion", 100);
			legMA_pub = n.advertise<std_msgs::Int32MultiArray>("leg_MA", 100);
		
		}
		twowheel VelocityController(double v, double w);
		void Body_speed_control(twowheel speed);
		void speed_publish(int v1, int v2);
		void rm_publish(int rm);
		void vw_control(const geometry_msgs::Twist vector);
		void leg_LA_publish(int v1, int v2);
	    ros::NodeHandle n;
		ros::Subscriber vw_sub;
		ros::Publisher  RS_pub;
		ros::Publisher  RM_pub;
		ros::Publisher legMA_pub;

};


twowheel Troller::VelocityController(double v, double w) // v,w ->Two wheel vel 
{

	speed.left = v - (W / 2)*w;//v left
	speed.right = v + (W / 2)*w;//v right
	std::cout << "v: " << v << " w: " << w << std::endl;
	return speed;
}
void Troller:: Body_speed_control(twowheel speed) //Two wheel vel to rpm
{
	M2 = (int)(speed.left *Ratio * 60 / (2 * PI*R));//Convert m/s into rpm
	M3 = (int)(speed.right *Ratio * 60 / (2 * PI*R));//Convert m/s into rpm
	//std::cout << "M2: " << M2 << " M3: " << M3 << std::endl;
}

//----Publish Function---
void Troller:: speed_publish(int v1,int v2)
{
	rs[0] = v1;
	rs[1] = v2;

		std_msgs::Int32MultiArray robot_speed;

		robot_speed.data.clear();
		robot_speed.data.push_back(rs[0]);
		robot_speed.data.push_back(rs[1]);

		RS_pub.publish(robot_speed);

		//printf("Robot_speed has published ...\n");

		ros::spinOnce();


}
void Troller:: rm_publish(int rm)
{

		std_msgs::Int32 robot_motion;
		robot_motion.data = rm;
		RM_pub.publish(robot_motion);
		//printf("Robot_motion has published ...\n");

		ros::spinOnce();

}
void Troller::leg_LA_publish (int position1, int position2)
{


	
	std_msgs::Int32MultiArray leg_pub;
	
	leg_pub.data.clear();
	
	lMA[0] = position1;
	lMA[1] = position2;

	for (int i = 0; i < 2; i++)
	{
		leg_pub.data.push_back(lMA[i]);
	}

	legMA_pub.publish(leg_pub);

	//printf("Robot_leg_MA has published ...\n");

	ros::spinOnce();


}
//----Publish Function---

//----Control v & w----
void Troller::vw_control(const 	geometry_msgs::Twist vector)//subscribe the signal from  Tracked gogo
{
	if( !n.getParam("odom_R", R) )
    ROS_ERROR("Failed to get odom_R from server.");
	if( !n.getParam("odom_W", W) )
    ROS_ERROR("Failed to get odom_W from server.");

//	if (ctr == 0)
//		leg_LA_publish(-26000,-27000);
//	ctr++;
    float vmax = 0.3;
    float wmax = 1.0;

	std::cout<<"v_true: "<<vector.linear.x<<" w_true: "<<vector.angular.z<<std::endl;
	if (vector.linear.x <= vmax&&vector.linear.x >= -vmax)//limit max speed
		s.v = vector.linear.x;
	else if (vector.linear.x < -vmax)
		s.v = -vmax;
	else
		s.v = vmax;

	if (vector.angular.z <= wmax&&vector.angular.z >= -wmax)//limit max orientation
		s.w = vector.angular.z;
	else if (vector.angular.z < -wmax)
		s.w = -wmax;
	else
		s.w = wmax;

	if (s_old.v != s.v || s_old.w != s.w) //prevent garbage publishing
	{
		speed = VelocityController(s.v, s.w);//Transform into two wheel velocity
		Body_speed_control(speed);		 //Transform into Motor rpm
		speed_publish(M2, M3);			 //Publish speed
		rm_publish(1);					 //Publish motion as forward.
	}
	ros::spinOnce();

	s_old.v = s.v;//update v 
	s_old.w = s.w;//update w

}
//----Control v & w----

int main(int argc, char **argv)
{

	ros::init(argc, argv, "Troller");
	Troller Troller;
	Troller.n;

	// Troller.leg_LA_publish(-26000,-27000);
	
	// ros::spinOnce;

	while (ros::ok())
	{
		ros::spinOnce();
	}

	return 0;
}


