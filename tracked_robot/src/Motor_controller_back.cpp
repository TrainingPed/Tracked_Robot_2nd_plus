#include "ros/ros.h"
#include <cstdio>
#include <cstdlib>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <errno.h>
#include <string.h>
#include <iostream> //lib use in c++
#include "conio.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"
#include <time.h>


#include "TrackedRobot_MotorHeader/MotorSet.h"
#include "TrackedRobot_MotorHeader/Slim5_CS.h"
#include "TrackedRobot_MotorHeader/Slim5_Speed.h"
#include "TrackedRobot_MotorHeader/Encoder.h"
#include <vector>

int rs[2];//robot_speed
int ls[2];//leg_speed
int rMA[2];//robot_MA
int lMA[2];//leg_MA
int rVA[2];//robot_VA
int lVA[2];//leg_VA
int rHO[2];//robot_HO
int lHO[2];//leg_HO
int rMR[2];//robot_MR

int rm=0;//robot_motion
int lm=0;//leg_motion

std::vector <int> encoder1;
std::vector <int> encoder2;
std::vector <int> encoder3;
std::vector <int> encoder4;
void pop_front(std::vector <int> *vec,int Cap)
{
	while(vec->size()>Cap)//Check if vector size is bigger than capacity
	{
		vec->erase(vec->begin());//delete the front element
	}
}

void robot_speed(const std_msgs::Int32MultiArray::ConstPtr& array)
{
	//ROS_INFO("robot_speed");
	rs[0]=*(array->data.begin());
	rs[1]=*(array->data.begin()+1);
	ROS_INFO("robot_speed change :L=%d,R=%d",rs[0],rs[1]);
	bodyJOGvel(rs[0],rs[1]);//[left,right]	
}
void leg_speed(const std_msgs::Int32MultiArray::ConstPtr& array)
{
	//ROS_INFO("leg_speed");
	ls[0]=*(array->data.begin());
	ls[1]=*(array->data.begin()+1);
	ROS_INFO("leg_speed change:F=%d,B=%d",ls[1],ls[0]);//ls[0]:back,ls[1]:front
	legJOGvel(ls[0],ls[1]);	//(back,front)
}
void robot_motion(const std_msgs::Int32::ConstPtr& msg)
{
	//ROS_INFO("robot_motion");
	rm=msg->data;
	ROS_INFO("robot_motion:%d",rm);
	
	if(rm==0)
	{
		bodyact("stop");

	}
	else if(rm==1)
	{
		bodyact("fwd");
	}
	else if(rm==2)
	{
		bodyact("back");
	}
	else if(rm==3)
	{
		bodyact("left");
	}
	else if(rm==4)
	{
		bodyact("right");
	}
	usleep(2000);	
}
void leg_motion(const std_msgs::Int32::ConstPtr& msg)
{
	//ROS_INFO("robot_motion");
	lm=msg->data;
	ROS_INFO("leg_motion:%d",lm);
	
	
	if(lm==1)
	{
		legact("","rev");
	}
	else if(lm==2)
	{
		legact("","fwd");
	}
	else if(lm==3)
	{
		legact("rev","");
	}
	else if(lm==4)
	{
		legact("fwd","");
	}
	usleep(2000);	
}
void robot_MA(const std_msgs::Int32MultiArray::ConstPtr& array)
{
	rMA[0]=*(array->data.begin());
	rMA[1]=*(array->data.begin()+1);
	ROS_INFO("robot_MA:L=%d,R=%d",rMA[0],rMA[1]);
	bodyma(rMA[0],rMA[1]);//[left,right]	
}
void robot_MR(const std_msgs::Int32MultiArray::ConstPtr& array)
{
	rMR[0]=*(array->data.begin());
	rMR[1]=*(array->data.begin()+1);
	ROS_INFO("robot_MR:L=%d,R=%d",rMR[0],rMR[1]);
	bodymr(rMR[0],rMR[1]);//[left,right]	
}
void robot_VA(const std_msgs::Int32MultiArray::ConstPtr& array)
{
	rVA[0]=*(array->data.begin());
	rVA[1]=*(array->data.begin()+1);
	ROS_INFO("robot_VA:L=%d,R=%d",rVA[0],rVA[1]);
	bodyva(rVA[0],rVA[1]);//[left,right]	
}
void robot_HO(const std_msgs::Int32MultiArray::ConstPtr& array)
{
	rHO[0]=*(array->data.begin());
	rHO[1]=*(array->data.begin()+1);
	ROS_INFO("robot_HO:L=%d,R=%d",rHO[0],rHO[1]);
	bodycs(rHO[0],rHO[1]);//[left,right]	
}
void leg_MA(const std_msgs::Int32MultiArray::ConstPtr& array)
{
	lMA[0]=*(array->data.begin());
	lMA[1]=*(array->data.begin()+1);
	ROS_INFO("leg_MA:F=%d,B=%d",lMA[1],lMA[0]);//ls[0]:back,ls[1]:front
	legma(lMA[0],lMA[1]);	//(back,front)
}
void leg_VA(const std_msgs::Int32MultiArray::ConstPtr& array)
{
	lVA[0]=*(array->data.begin());
	lVA[1]=*(array->data.begin()+1);
	ROS_INFO("leg_VA:F=%d,B=%d",lVA[1],lVA[0]);//ls[0]:back,ls[1]:front
	legva(lVA[0],lVA[1]);	//(back,front)
}
void leg_HO(const std_msgs::Int32MultiArray::ConstPtr& array)
{
	lHO[0]=*(array->data.begin());
	lHO[1]=*(array->data.begin()+1);
	ROS_INFO("leg_HO:F=%d,B=%d",lHO[1],lHO[0]);//ls[0]:back,ls[1]:front
	legcs(lHO[0],lHO[1]);	//(back,front)
}
void ReadEnc(const std_msgs::Int32::ConstPtr& msg)
{
	clock_t t;
	t=clock();
	
	int enc = 0;
	enc = msg->data;
	//ROS_INFO("Reading encoder ..." );
	if (enc == 1)
	{
		encoder1.push_back(Encoder(1, fd_Slim5));
		usleep(2000);
		encoder2.push_back(Encoder(2, fd_Slim5));
		usleep(2000);
		encoder3.push_back(Encoder(3, fd_Slim5));
		usleep(2000);
		encoder4.push_back(Encoder(4, fd_Slim5));
		pop_front(&encoder1,1000);
		pop_front(&encoder2,1000);
		pop_front(&encoder3,1000);
		pop_front(&encoder4,1000);
	}
	t = clock() - t;
	printf("It took me (%f seconds).\n", ((float)t) / CLOCKS_PER_SEC);
	std::cout << "Encoder 1: " << encoder1.back() << " Encoder 2: " << encoder2.back() << std::endl;
	std::cout << "Encoder 3: " << encoder3.back() << " Encoder 4: " << encoder4.back() << std::endl;
	
}

/*
void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  int counter = 18;
  //ROS_INFO("No. %d ranges size is :%d", counter, scan_msg->ranges.size());
  for(int i = 0 ; i < 721; i+=39)
  {
	 //ROS_INFO("%f", scan_msg->ranges[i]);
	 //printf("%4.2lf\n",Laserscan[counter]);
	 Laserscan[counter]=(scan_msg->ranges[i])*100;
	 
	 counter--;
  }
  
}
*/


int main(int argc, char **argv)
{
//=======ROS_init=========================================
	
   ros::init(argc, argv, "Motor_controller");
   ros::NodeHandle n;
   ros::Subscriber sub1 = n.subscribe("robot_speed", 100, robot_speed);
   ros::Subscriber sub2 = n.subscribe("leg_speed", 100, leg_speed);
   ros::Subscriber sub3 = n.subscribe("robot_MA", 100, robot_MA);
   ros::Subscriber sub4 = n.subscribe("leg_MA", 100, leg_MA);
   ros::Subscriber sub5 = n.subscribe("robot_HO", 100, robot_HO);
   ros::Subscriber sub6 = n.subscribe("robot_VA", 100, robot_VA);
   ros::Subscriber sub7 = n.subscribe("robot_motion", 100, robot_motion);
   ros::Subscriber sub8 = n.subscribe("leg_motion", 100, leg_motion);
   ros::Subscriber sub9 = n.subscribe("leg_HO", 100, leg_HO);
   ros::Subscriber sub10 = n.subscribe("leg_VA", 100, leg_VA);
   ros::Subscriber sub11 = n.subscribe("robot_MR", 100, robot_MR);

   ros::Publisher  pubENC = n.advertise<std_msgs::Int32MultiArray>("Rd_enc2", 10000);
   ros::Subscriber subENC = n.subscribe("Rd_enc", 10000, ReadEnc);
   

	fd_Slim5 = open(serialPortName, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
	if(Motor_initial(fd_Slim5) < 0) //  Check Motor Open
   {
		printf("End\n");
		return 0;
   }
	//---Set JG vel
	bodyJOGvel(1000,1000);
	usleep(2000);
   legJOGvel(200,200);
	usleep(2000);
   //---Set MA vel
	bodyMAvel(700,700); //700
   usleep(2000);
	legMAvel(200,200);  //200
	usleep(2000);
   //---Set CS
	legcs(0,0);
	usleep(2000);
	bodycs(0,0); //0,0   initial angle
	usleep(2000);
   //---Set VA
	bodyva(300,300); //set Acceleration : 900, 500(¥[³t«×)
	//legva(600,600);
   usleep(2000);	

   encoder1.push_back(0);
   encoder2.push_back(0);
   encoder3.push_back(0);
   encoder4.push_back(0);

   ros::spinOnce();
   

   ros::Rate r(5); // 5 hz=0.2sec

   while (ros::ok())
   {
	   std_msgs::Int32MultiArray Encoders;

	   Encoders.data.clear();
	   
	   Encoders.data.push_back(encoder1.back());
	   Encoders.data.push_back(encoder2.back());
	   Encoders.data.push_back(encoder3.back());
	   Encoders.data.push_back(encoder4.back());

	   pubENC.publish(Encoders);
	   ros::spinOnce();
	   r.sleep();
   }
	return 0;
}

