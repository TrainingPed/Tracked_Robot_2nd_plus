#include "ros/ros.h"
#include <stdlib.h>
#include <stdio.h>


#include "UpStairMode.h"
#include "DownStairMode.h"





int main(int argc,char **argv)
{
    ros::init(argc,argv,"UTP_Demo");

              UpStairMode USM;
              DownStairMode DSM;

              USM.n;
              DSM.n;

              printf("Demo start\n");

              /*----------------------------------*/
//              printf("Motor Test ! \n");
//              UpStairMode().legspeed(3000,3000);
//              while(ros::ok)
//              {
//                  UpStairMode().legmotion("Front_Up");
//              }
//              printf("Running here ...\n");
//              sleep(1000);
//              sleep(1000);
//              sleep(1000);
//              sleep(1000);

              /*----------------------------------*/

              printf("Start UpStairMode !\n");

              UpStairMode().legspeed(200,200);
              usleep(300000);
              UpStairMode().legMA(26618,26454);
              usleep(300000);
              UpStairMode().legMA_feedback(27000,26000);
              usleep(300000);

              printf("start mode_23\n");
              UpStairMode().mode_23(0.05,0.43,0.43,0.43,false);
              printf("start mode_3\n");
              UpStairMode().mode_3();
              printf("start mode_4\n");
              UpStairMode().mode_4();
              usleep(200000);
              UpStairMode().vw_control(0.05,0);
              sleep(3);
              UpStairMode().vw_control(0,0.2);
              sleep(10);
              usleep(700000);


              //Remember to fill in laser align


              UpStairMode().mode_align();
                    /*code*/


//              UpStairMode().robotmotion("Stop");

              printf("Start ownStairMode !\n");

              DownStairMode().check_us();

              printf(" ------ start mode_6 ------ \n");
              DownStairMode().mode_6();

              //climb downstair and land
              printf(" ------ start mode_7 ------ \n");
              DownStairMode().mode_7();

              DownStairMode().robotmotion("Stop");
              usleep(300000);

              printf("End of Demo !\n");

              return 0;
}
