#include "ros/ros.h"

#include <stdlib.h>
#include <stdio.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "DownStairMode.h"

#define width 640
#define height 480

int main(int argc, char **argv)
{

	ros::init(argc, argv, "DownStairMode");

	DownStairMode DSM;

	DSM.n;

    ros::Rate r(1);

	// while(ros::ok())
	// {
	// 	if (DSM.d<1000)break;
	// 	ros::spinOnce();
	// 	r.sleep();
	// }


	std::cout<<"Prepare to go Downstairs!!\n";
    usleep(300000);
    while(ros::ok()&&!DSM.safe_switch)
	{
        ros::spinOnce();
		printf("\n");
        printf("Checking ultrasonic data ......\t\t us = %d\n",DSM.us);

		//printf("d21=%d\n",USM.d21);
        if(DSM.us!=0)
        {
            printf("Connect to ultrasonic successful ! \n\n");
			break;
        }
        printf("Fail ! Retry Once ! \n");
		r.sleep();
	}
//	sleep(3);
//	DSM.legMA(-26000,-27000);
    sleep(3);
	// DSM.mode_5();

    //get close to the stair and lean over
    printf(" ------ start mode_6 ------ \n");
    DSM.mode_6();

    //climb downstair and land
    printf(" ------ start mode_7 ------ \n");
    DSM.mode_7();
//    int i=0;
//    while(ros::ok())
//    {
//        ros::spinOnce();
//            DSM.vw_control(0,0.04);
//            usleep(300000);
//            DSM.vw_control(0,-0.04);
//            usleep(300000);
//            printf("%d\n",i++);

//    }


    DSM.vw_control(0,0.2);
    sleep(11);
    usleep(100000);
    DSM.robotmotion("Stop");
    usleep(300000);

//    caliberate the motor by servo off
//    DSM.motor_switch(1,0);
//    usleep(500000);
//    DSM.motor_switch(4,0);
//    usleep(500000);
//    sleep(1);
//    DSM.motor_switch(1,1);
//    usleep(500000);
//    DSM.motor_switch(4,1);
//    usleep(500000);

	return 0;
}



