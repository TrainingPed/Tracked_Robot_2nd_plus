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
#include <pthread.h>



#include "TrackedRobot_MotorHeader/MotorSet.h"
#include "TrackedRobot_MotorHeader/Slim5_fox.h"
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
int body_max_speed = 700;
int leg_max_speed = 200;


pthread_mutex_t mutex;

std::vector <int> encoder1;
std::vector <int> encoder2;
std::vector <int> encoder3;
std::vector <int> encoder4;

void * readencoder(void *ptr);

void pop_front(std::vector <int> *vec,int Cap)
{
	
	while(vec->size()>Cap && ros::ok())//Check if vector size is bigger than capacity
	{
		std::cout << "pop front ..." << std::endl;
		vec->erase(vec->begin());//delete the front element
	}
}

void robot_speed(const std_msgs::Int32MultiArray::ConstPtr& array)
{
		pthread_mutex_lock(&mutex); // mutex lock on
		rs[0] = *(array->data.begin());
		rs[1] = *(array->data.begin() + 1);
		ROS_INFO("Body_speed :Left = %d, Right = %d", rs[0], rs[1]);
		Motor_speed(2, rs[0]);//left
		Motor_speed(3, rs[1]);//right
		pthread_mutex_unlock(&mutex);//lock off
		usleep(2000);//delay for awhile
}
void leg_speed(const std_msgs::Int32MultiArray::ConstPtr& array)
{
		pthread_mutex_lock(&mutex); // mutex lock on
		ls[0] = *(array->data.begin());
		ls[1] = *(array->data.begin() + 1);
		ROS_INFO("Leg_speed : Front = %d, Back =%d", ls[1], ls[0]);
		Motor_speed(4, ls[0]);//back
		Motor_speed(1, ls[1]);//front
		pthread_mutex_unlock(&mutex);//lock off
		usleep(2000);//delay for awhile

}
void robot_motion(const std_msgs::Int32::ConstPtr& msg)
{
	//ROS_INFO("robot_motion");
		pthread_mutex_lock(&mutex); // mutex lock on
		rm = msg->data;
		ROS_INFO("Robot_motion: %d ", rm);

		switch (rm)
		{
		case 0:
			ROS_INFO("Stop!");
			Motor_vel(1, 0);//front
			Motor_vel(4, 0);//back
			Motor_vel(2, 0);//left
			Motor_vel(3, 0);//right
			break;
		case 1:
			ROS_INFO("Move Foward!");
			Motor_vel(2, -body_max_speed);//left
			Motor_vel(3, body_max_speed);//right
			break;
		case 2:
			ROS_INFO("Move Backward!");
			Motor_vel(2, body_max_speed);//left
			Motor_vel(3, -body_max_speed);//right
			break;
		case 3:
			ROS_INFO("Move Left!");
			Motor_vel(2, body_max_speed);//left
			Motor_vel(3, body_max_speed);//right
			break;
		case 4:
			ROS_INFO("Move Right!");
			Motor_vel(2, -body_max_speed);//left
			Motor_vel(3, -body_max_speed);//right
			break;
		default:
			break;
			
		}
		pthread_mutex_unlock(&mutex);//lock off
		usleep(2000);//delay for awhile
}
void leg_motion(const std_msgs::Int32::ConstPtr& msg)
{
		pthread_mutex_lock(&mutex); // mutex lock on
		lm = msg->data;
		ROS_INFO("Leg_motion: %d ", lm);
		switch (lm)
		{
		case 1:
			ROS_INFO("Front leg Upward!");
			Motor_vel(1, leg_max_speed);//front
			break;
		case 2:
			ROS_INFO("Front leg Downward!");
			Motor_vel(1, -leg_max_speed);//front
			break;
		case 3:
			ROS_INFO("Back leg Upward!");
			Motor_vel(4, leg_max_speed);//back
			break;
		case 4:
			ROS_INFO("Back leg Downward!");
			Motor_vel(4, -leg_max_speed);//back
			break;
		default:
			break;
		}
		pthread_mutex_unlock(&mutex);//lock off
		usleep(2000);//delay for awhile
}
void robot_MA(const std_msgs::Int32MultiArray::ConstPtr& array)
{
		pthread_mutex_lock(&mutex); // mutex lock on
		rMA[0] = *(array->data.begin());
		rMA[1] = *(array->data.begin() + 1);
		ROS_INFO("Robot_LA : Left = %d, Right = %d ", rMA[0], rMA[1]);
		Motor_speed(2, body_max_speed);//left
		Motor_speed(3, body_max_speed);//right
		Motor_LA(2, rMA[0]);
		Motor_LA(3, rMA[1]);
		Motor_M(2);
		Motor_M(3);
		pthread_mutex_unlock(&mutex);//lock off
		usleep(2000);//delay for awhile	
	
}
void robot_MR(const std_msgs::Int32MultiArray::ConstPtr& array)
{
		pthread_mutex_lock(&mutex); // mutex lock on
		rMR[0] = *(array->data.begin());
		rMR[1] = *(array->data.begin() + 1);
		ROS_INFO("Robot_LR : Left = %d, Right = %d ", rMR[0], rMR[1]);
		Motor_speed(2, body_max_speed);//left
		Motor_speed(3, body_max_speed);//right
		Motor_LR(2, rMA[0]);
		Motor_LR(3, rMA[1]);
		Motor_M(2);
		Motor_M(3);
		pthread_mutex_unlock(&mutex);//lock off
		usleep(2000);//delay for awhile			

	
}
void robot_VA(const std_msgs::Int32MultiArray::ConstPtr& array)
{
		pthread_mutex_lock(&mutex); // mutex lock on
		rVA[0] = *(array->data.begin());
		rVA[1] = *(array->data.begin() + 1);
		ROS_INFO("Robot_A/D : Left = %d, Right = %d ", rVA[0], rVA[1]);
		Motor_acc(2, rVA[0]);//left 
		Motor_acc(3, rVA[1]);//right
		Motor_dec(2, rVA[0]);
		Motor_dec(3, rVA[1]);
		pthread_mutex_unlock(&mutex);//lock off
		usleep(2000);//delay for awhile			
	
}
void robot_HO(const std_msgs::Int32MultiArray::ConstPtr& array)
{
		pthread_mutex_lock(&mutex); // mutex lock on
		rHO[0] = *(array->data.begin());
		rHO[1] = *(array->data.begin() + 1);
		ROS_INFO("Robot_HO : Left = %d, Right = %d ", rHO[0], rHO[1]);
		Motor_HO(2, rHO[0]);//left
		Motor_HO(3, rHO[1]);//right
		pthread_mutex_unlock(&mutex);//lock off
		usleep(2000);//delay for awhile			

}
void leg_MA(const std_msgs::Int32MultiArray::ConstPtr& array)
{
		pthread_mutex_lock(&mutex); // mutex lock on	
		lMA[0] = *(array->data.begin());
		lMA[1] = *(array->data.begin() + 1);
		ROS_INFO("Leg_MA : Front = %d, Back =%d", lMA[1], lMA[0]);
		Motor_speed(1, leg_max_speed);//left
		Motor_speed(4, leg_max_speed);//right
		Motor_LA(1, -lMA[0]);
		Motor_LA(4, -lMA[1]);
		Motor_M(1);
		Motor_M(4);
		pthread_mutex_unlock(&mutex);//lock off
		usleep(2000);//delay for awhile			
	
}
void leg_VA(const std_msgs::Int32MultiArray::ConstPtr& array)
{
		pthread_mutex_lock(&mutex); // mutex lock on	
		lVA[0] = *(array->data.begin());
		lVA[1] = *(array->data.begin() + 1);
		ROS_INFO("Leg_A/D: : Front = %d, Back =%d", lVA[1], lVA[0]);
		Motor_acc(4, lVA[0]);//back
		Motor_acc(1, lVA[1]);//front
		Motor_dec(4, lVA[0]);
		Motor_dec(1, lVA[1]);
		pthread_mutex_unlock(&mutex);//lock off
		usleep(2000);//delay for awhile	
	
}
void leg_HO(const std_msgs::Int32MultiArray::ConstPtr& array)
{
		
		pthread_mutex_lock(&mutex); // mutex lock on
		lHO[0] = *(array->data.begin());
		lHO[1] = *(array->data.begin() + 1);
		ROS_INFO("Leg_HO : Left = %d, Right = %d ", lHO[1], lHO[0]);
		Motor_HO(4, lHO[0]);//back
		Motor_HO(1, lHO[1]);//front
		pthread_mutex_unlock(&mutex);//lock off
		usleep(2000);//delay for awhile
	
}

void *readencoder(void *ptr)
{
	clock_t t;
	t = clock();

	pthread_mutex_lock(&mutex); // mutex lock on

								//ROS_INFO("Reading encoder ..." );


		encoder1.push_back(Encoder(1, fd_Slim5));
		usleep(2000);
		encoder2.push_back(Encoder(2, fd_Slim5));
		usleep(2000);
		encoder3.push_back(Encoder(3, fd_Slim5));
		usleep(2000);
		encoder4.push_back(Encoder(4, fd_Slim5));
		//pop_front(&encoder1,10000);
		//pop_front(&encoder2,10000);
		//pop_front(&encoder3,10000);
		//pop_front(&encoder4,10000);


	t = clock() - t;
	/*printf("It took me (%f seconds).\n", ((float)t) / CLOCKS_PER_SEC);
	std::cout << "Encoder 1: " << encoder1.back() << " Encoder 2: " << encoder2.back() << std::endl;
	std::cout << "Encoder 3: " << encoder3.back() << " Encoder 4: " << encoder4.back() << std::endl;*/

	pthread_mutex_unlock(&mutex);//lock off
	usleep(2000);//delay for awhile
}



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
   //ros::Subscriber subENC = n.subscribe("Rd_enc", 10000, ReadEnc);
   

	fd_Slim5 = open(serialPortName, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
	if(Motor_initial(fd_Slim5) < 0) //  Check Motor Open
   {
		printf("End\n");
		return 0;
   }
	//---Set JG vel
	Motor_speed(2, body_max_speed);//left
	Motor_speed(3, body_max_speed);//right
	Motor_speed(1, leg_max_speed);//front
	Motor_speed(4, leg_max_speed);//back

   //---Set CS
	Motor_HO(1, 0);
	Motor_HO(2, 0);
	Motor_HO(3, 0);
	Motor_HO(4, 0);//Initial Home
	//usleep(2000);
   //---Set VA
	Motor_acc(2, 300);
	Motor_acc(3, 300);
   //usleep(2000);	

   encoder1.push_back(0);
   encoder2.push_back(0);
   encoder3.push_back(0);
   encoder4.push_back(0);

   //thread setting 
   pthread_t thread;

   //define delay time


   //Initialize a mutex
   int *enc = 0;
   pthread_mutex_init(&mutex, NULL);
  
  
   ros::Rate r(5); // 5 hz=0.2sec

   while (ros::ok())
   {
	   pthread_create(&thread, NULL, &readencoder, NULL);
	   //first param point to thread name pointer; second param is used to set the propertiy of thread;thrid param is the start address of function we want to run;last one is the param of function

	   std_msgs::Int32MultiArray Encoders;

	   Encoders.data.clear();
	   
	   Encoders.data.push_back(encoder1.back());
	   Encoders.data.push_back(encoder2.back());
	   Encoders.data.push_back(encoder3.back());
	   Encoders.data.push_back(encoder4.back());

	   pubENC.publish(Encoders); 
	   pthread_join(thread, NULL);
	   ros::spinOnce(); // Sub 
	   r.sleep();
   }
	return 0;
}

