#include "ros/ros.h"

#include <stdlib.h>
#include <stdio.h>
 
#include <sensor_msgs/image_encodings.h> 
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "UpStairMode.h"



int main(int argc, char **argv)
{

	ros::init(argc, argv, "UpStairMode");
	
	UpStairMode USM;
	
    USM.n;
	
		
	
	ros::Rate r(2);
	
//    while(ros::ok())
//    {
////		std::cout<<"d21="<<USM.d21<<" cnt21="<<USM.counter21<<"\n";
////		if(USM.d21!=0 && USM.counter21>70)break;
//        //printf("d21=%d\n",USM.d21);

//        std::cout<<"pitch="<<USM.pitch*180/3.14<<" \n";
//        std::cout<<"roll="<<USM.roll*180/3.14<<" \n";
//        std::cout<<"yaw="<<USM.yaw*180/3.14<<" \n\n";
//        ros::spinOnce();
//        r.sleep();
//    }
//    USM.start();
    printf ("start initializing\n");
//    Initial ready pose
    USM.legspeed(300,230);
    usleep(300000);
    USM.legMA_feedback(38000,29000);
    usleep(300000);

//    USM.vw_control(0.05,0.01);
//    sleep(10);
//    USM.robotmotion("Stop");
//    sleep(40);

//    USM.mode_21();

//    USM.mode_23(0.05,0.5,0.45,0.7,false);

    //get close to the stair and lean on
    printf("start mode_23\n");
    USM.mode_23(0.05,0.6,0.4,0.6,false);

    //get on to the stair
    printf("start mode_3\n");
    USM.mode_3();

    //climbing over and land
    sleep(2);
    printf("start mode_4\n");

//    while(ros::ok())
//    {

//        ros::spinOnce();
//        if(USM.depth_scan_hit( 95, 95 ))
//        {
//            USM.robotmotion("Stop");
//            printf("Ready to put down arm!!");
//            usleep(300000);
//            break;
//        }
//        else
//        {

//            printf("Keep climbing");
//        }
//    }
    USM.mode_4();
    usleep(200000);
    USM.vw_control(0,0.2);
    sleep(8);
    usleep(100000);
   double align_laser;
   while(ros::ok())
   {
       align_laser = USM.Align180();
        printf("%f\n",align_laser);
       if( align_laser>=1.2 )//  need to turn left
       {
            USM.vw_control(0,0.01);
            usleep(300000);
            printf("Turn Left\n");
       }
       else if( align_laser<= 1)// need to turn right
       {
            USM.vw_control(0,-0.01);
            usleep(300000);
            printf("Turn Right\n");
       }
       else
       {
           USM.robotmotion("Stop");
           printf("OK\n");
           break;
       }

   }


    USM.robotmotion("Stop");

//    caliberate the motor by servo off



    //initial zero position
//    USM.motor_switch(1,0);
//    usleep(500000);
//    USM.motor_switch(4,0);
//    usleep(500000);
//    sleep(1);
//    USM.motor_switch(1,1);
//    usleep(500000);
//    USM.motor_switch(4,1);
//    usleep(500000);



	return 0;
}



