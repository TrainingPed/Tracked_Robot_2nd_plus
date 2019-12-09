#include "ros/ros.h"
#include <cstdio>
#include <cstdlib>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <errno.h>
#include <cstdio>
#include <iostream> //lib use in c++
#include <time.h>
#include "conio.h"

/*---Header Files---*/
//OpenNI
//#include "TrackedRobot_ActionHrader/OpenniSet.h" //OpenNI Setting

//Tracked Robot Motor Control
#include "TrackedRobot_MotorHeader/MotorSet.h"
#include "TrackedRobot_MotorHeader/Slim5_CS.h"
//#include "TrackedRobot_MotorHeader/Slim5_JG.h"
//#include "TrackedRobot_MotorHeader/Slim5_MA.h"
//#include "TrackedRobot_MotorHeader/Slim5_MR.h"
#include "TrackedRobot_MotorHeader/Slim5_Speed.h"
//#include "TrackedRobot_MotorHeader/Slim9_XtionMotor.h"
//#include "TrackedRobot_MotorHeader/MotorEncoder.h"
//#include "TrackedRobot_MotorHeader/MotorEncoder_Xtion.h"

//Tracked Robot Mission
//#include "TrackedRobot_ActionHrader/Fuzzy.h"
//#include "TrackedRobot_ActionHrader/XtionDepth.h"
//#include "TrackedRobot_ActionHrader/WallFollowing.h"
//#include "TrackedRobot_ActionHrader/OtherFunction.h"
//#include "TrackedRobot_ActionHrader/UpStairsMode.h"
//#include "TrackedRobot_ActionHrader/DownStairsMode.h"
//#include "TrackedRobot_ActionHrader/ObstacleAvoidance.h"
#include "TrackedRobot_ActionHrader/TR_ManualControl.h"

    int ob_time = 0;

int main( int argc, char** argv )
{
/*---Motor initialize set---*/

    if(Motor_initial() < 0) //  Check Motor Open
    {
        printf("End\n");
        return 0;
    }



    //---Set JG vel
	bodyJOGvel(1000,1000);
	usleep(10000);
    legJOGvel(200,200);
	usleep(10000);
    //---Set MA vel
	bodyMAvel(700,700); //700
    usleep(10000);
	legMAvel(200,200);  //200
	usleep(10000);
    //---Set CS
	//legcs(0,0);
	usleep(10000);
	bodycs(0,0); //0,0   initial angle
	usleep(10000);
    //---Set VA
	bodyva(300,300); //set Acceleration : 900, 500(¥[³t«×)
	//legva(600,600);
    usleep(10000);

    //---Climb stairs leg set 90 Degree
    //legma(-30000,-28500); //climb stairs legs initial
    //XtionMotor(900,0);



/*---Tracked Robot Manual Control---*/

    if(TrackRobot_ManualControl() < 0)
    {
        printf("TrackRobot_ManualControl End\n");
        return 0;
    }


///---stop up stsor---
    Motor_close();




	printf("Thanks for your attention!\n");

	return 0;

}
