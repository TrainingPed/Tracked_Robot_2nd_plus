#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/String.h"
#include <iostream> 
#include <vector>

#define ARRAYSIZE 1000
std::vector <int> Encoders;

void pop_front(std::vector <int> *vec , int Cap)
{
	while (vec->size()>Cap)//Check if vector size is bigger than capacity
	{
		//std::cout << "vector size: " << vec->size();
		//std::cout << "pop front ..." << std::endl;
		vec->erase(vec->begin());//delete the front element
	}
}
void ReadEnc2(const std_msgs::Int32MultiArray::ConstPtr& array)
{


	for (int j = 0; j < 4; j++)
	{
		Encoders.push_back(*(array->data.begin() + j));
/*		std::cout << "Encoder " << j << " is " << Encoders.at(j) << std::endl*/;
	}
	pop_front(&Encoders, 4);
	std::cout << "Encoder 1: " << Encoders.at(0) << " Encoder 2: " << Encoders.at(1) << std::endl;
	std::cout << "Encoder 3: " << Encoders.at(2) << " Encoder 4: " << Encoders.at(3) << std::endl << std::endl;
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "encoderlistener");

	ros::NodeHandle n;
	ros::Publisher enc_pub = n.advertise<std_msgs::Int32>("Rd_enc", 100);
	ros::Subscriber enc_sub = n.subscribe("Rd_enc2", 100, ReadEnc2);
	ros::Rate r(2);
	while (ros::ok())
	{
		int i = 1;
		std_msgs::Int32 encoder;

			int enc = i;//encoder number
			encoder.data = enc;
			enc_pub.publish(encoder);

			ros::spinOnce();

		r.sleep();
	}
	return 0;
}