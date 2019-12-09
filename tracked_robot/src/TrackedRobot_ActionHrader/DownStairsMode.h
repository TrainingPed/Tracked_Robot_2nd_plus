/*---Down Stairs Program---*/

void mode_5();	/*---Before 'down' stairs alignment---*/
int mode_6();	/*---'down' stairs prepare----*/
void mode_7();	/*---'down' stairs alignment---*/


void mode_5()	//---Before 'down' stairs alignment
{
	/*---Time---*/
//    time_t t1 = time(NULL);
	/*---"depth_find" Function Variable---*/
	int i=0,j=0,x=0,y=0,mid_x=0,mid_y=0,sw,mid_x2=0,mid_y2=0;
	/*---Depth Data Variable---*/
	int d5=0,d51=0,d52=0;
	/*---For Down Stairs Alignment(Mode 5) Luck---*/
	int key=0,key_NoStair=0;
	/*---Out While---*/
	int d=1000;

    /*---Data Record - Open---*/
//    FILE *M5XD=fopen("/home/ubuntu/CodeBlocks/TrackedRobot_Control/Recore Data/M5_XtionDistance.txt","w");
//    FILE *M5PL=fopen("/home/ubuntu/CodeBlocks/TrackedRobot_Control/Recore Data/M5_PixelLocation.txt","w");
//    FILE *M5VL=fopen("/home/ubuntu/CodeBlocks/TrackedRobot_Control/Recore Data/M5_VelLeft.txt","w");
//    FILE *M5VR=fopen("/home/ubuntu/CodeBlocks/TrackedRobot_Control/Recore Data/M5_VelRight.txt","w");

    /*---Vel - For Record---*/
	int vel_L=0,vel_R=0;

    XtionMotor(550,0); //Put in mode 9
    usleep(2000000);

    printf("Mode 5 , Before down stairs Alignment \n");

    vel_L=350;
    vel_R=350;
    bodyJOGvel(vel_L,vel_R);
    //bodyJOGvel(350,350);

    bodyact("fwd");
    usleep(500000);

    while(d>650)
    {
        streamDepth.readFrame( &frameDepth );
        depth_find(5,&mid_x2,&mid_y2,&mid_x,&mid_y,&i,&j,&x,&sw);

        legact("stop","stop");
        usleep(50000);

        //fprintf(M5XD,"%d\n",depthMD(mid_x,mid_y));
        //fprintf(M5PL,"%d\n",mid_x);

        if(mid_x!=0 && mid_y!=0)
        {
            d51=(depthMD(x+30,j+30)+depthMD(x+30,j+20))/2;
            d52=(depthMD(i-30,sw+30)+depthMD(i-30,sw+20))/2;
            d5=abs(d51-d52);

            if(d5<=22) //25
            {
                printf("mode5 , Alignment OK!\n");
                //printf("d=%d\n",d);
//                bodyact("stop");
//                usleep(50000);

                //vel_L=300;
                //vel_R=300;
                //bodyJOGvel(vel_L,vel_R);
                //bodyJOGvel(300,300);

                bodyact("fwd");
                usleep(300000);
                break; //*out_mode=6; //mode here!
            }
            else if(key!=1 && d5>=23 && d51>d52 && d51!=0 && d52!=0) //26
            {
                //printf("d5=%d , Need Left\n",d5);
                key=1;
                bodyact("stop");
                usleep(50000);
                //printf("d=%d\n",d);

                vel_L=200;
                vel_R=200;
                bodyJOGvel(vel_L,vel_R);
                //bodyJOGvel(300,300);

                bodyact("left");
                usleep(100000);
                //bodyact("stop");
                //usleep(200000);
            }
            else if(key!=2 && d5>=23 && d51<d52 && d51!=0 && d52!=0) //26
            {
                //printf("d5=%d , Need Right\n",d5);
                key=2;
                bodyact("stop");
                usleep(50000);
                //printf("d=%d\n",d);

                vel_L=200;
                vel_R=200;
                bodyJOGvel(vel_L,vel_R);
                //bodyJOGvel(300,300);

                bodyact("right");
                usleep(100000);
                //bodyact("stop");
                //usleep(200000);
            }

            d=depthMD(mid_x,mid_y+5);
        }//if end
        else //can't find stairs
        {
            printf("Can't Find Stairs! 'Patorl Mode'\n");

//            vel_L=350;
//            vel_R=350;
//            bodyJOGvel(vel_L,vel_R);
//            //bodyJOGvel(350,350);
//
//            bodyact("fwd");
//            usleep(500000);
            //bodyact("stop");
            //usleep(5000);

            d=1000;
        }

        //fprintf(M5VL,"%d\n",vel_L);
        //fprintf(M5VR,"%d\n",vel_R);

    }

    /*---Time end---*/
//    time_t t2 = time(NULL);
    //fprintf(M5XD,"Mode5 Time = %d\n",t2-t1);

    /*---Data Record - Close---*/
//    fclose(M5XD);
//    fclose(M5PL);
//    fclose(M5VL);
//    fclose(M5VR);
}//mode5 end

int mode_6()	//---'down' stairs prepare
{
    /*---Time---*/
//    time_t t1 = time(NULL);

	/*---Data Record - open---*/
//    FILE *M6Time=fopen("/home/ubuntu/CodeBlocks/TrackedRobot_Control/Recore Data/M6_time.txt","w");

    printf("Mode 6 , Down stairs prepare \n");
    bodyJOGvel(350,350);
    usleep(50000);

    /*---Check Sensor Connection---*/

	int sensor_DATA01 = sensorData(1);


	if(sensor_DATA01<1)
	{
        printf("*** Sensor Disconnection ***\n\n");
        bodyact("stop");
        usleep(50000);
        return -1;
	}
	else
	{
	    printf("Ready to Close Stairs......\n");
	    bodyact("fwd");
        usleep(50000);
	}

    /*---If sensor have connection, Do---*/
    while(sensor_DATA01>1)
    {
        int sensor_DATA01 = sensorData(1);
        //fprintf(M6Time,"sensor = %d\n",sensor_DATA01);
        printf("HC sensor data = %d\n",sensor_DATA01);

        bodyact("fwd");
        usleep(50000);
        if(sensor_DATA01>15)
        {
            bodyact("stop");
            legact("stop","stop");
            usleep(50000);
            break;
        }

    }
    /*---Close Sensor---*/
    Sensor_close();


    /*---When close stairs---*/
    printf("Down Stairs Prepare.\n\n");
	//---Down Prepare
	bodyJOGvel(350,350);
	usleep(30000);//0.5
	legMAvel(200,200); //80,50
	usleep(30000);//0.5
	legcs(0,0);
	usleep(30000);

	legma(100*300,(90+angg)*300);
	//legma(100*300,(90+26)*300); //rang:20~40
	//Check_MotorPE(0,3);
	//sleep(5); //Check_MotorPE replace it.

	bodyact("fwd");
	usleep(3000000);
	legma(98*300,95*300);
	usleep(4500000); //5.5sec

	bodyact("stop");
	//*out_mode=7;

	/*---Time end---*/
//    time_t t2 = time(NULL);
    //fprintf(M6Time,"Mode6 Time = %d\n",t2-t1);

    /*---Data Record - close---*/
//    fclose(M6Time);

	return 1;

}//mode6 end

void mode_7()	//---'down' stairs alignment
{
	/*---Time---*/
//    time_t t1 = time(NULL);

	/*---"depth_find" Function Variable---*/
	int i=0,j=0,x=0,y=0,mid_x=0,mid_y=0,sw,mid_x2=0,mid_y2=0;
	/*---Depth Data Variable---*/
	int d7=0,d71=0,d72=0;
	/*---For Down Stairs Alignment(Mode 7) Luck---*/
	int key=0;
	/*---Out While---*/
	int d=1000;


    /*---Data Record - Open---*/
//    FILE *M7XD=fopen("/home/ubuntu/CodeBlocks/TrackedRobot_Control/Recore Data/M7_XtionDistance.txt","w");
//    FILE *M7PL=fopen("/home/ubuntu/CodeBlocks/TrackedRobot_Control/Recore Data/M7_PixelLocation.txt","w");
//    FILE *M7VL=fopen("/home/ubuntu/CodeBlocks/TrackedRobot_Control/Recore Data/M7_VelLeft.txt","w");
//    FILE *M7VR=fopen("/home/ubuntu/CodeBlocks/TrackedRobot_Control/Recore Data/M7_VelRight.txt","w");
//    FILE *M7clcr=fopen("/home/ubuntu/CodeBlocks/TrackedRobot_Control/Recore Data/M7_clcr.txt","w");
//    FILE *M7DD=fopen("/home/ubuntu/CodeBlocks/TrackedRobot_Control/Recore Data/M7_downDistance.txt","w");
    /*---Vel - For Record---*/
	int vel_L=0,vel_R=0;

    XtionMotor(600,0);
    usleep(2000000);

    printf("Mode 7 , Down stairs alignment \n");

    bodyact("fwd");
    usleep(30000);

    while(d>800)//700
    {
        streamDepth.readFrame( &frameDepth );
        depth_find(7,&mid_x2,&mid_y2,&mid_x,&mid_y,&i,&j,&x,&sw);

        //fprintf(M7XD,"%d\n",depthMD(320,80));
        //fprintf(M7PL,"%d\n",mid_x);
        //legact("stop","stop");
        //usleep(50000);
        bodyact("fwd");
        usleep(30000);

        if(mid_x!=0&&mid_y!=0)
        {
            d71=(depthMD(x+20,j+15)+depthMD(x+20,j+20)+depthMD(x+20,j+10))/3;
            d72=(depthMD(i-20,sw+15)+depthMD(i-20,sw+20)+depthMD(i-20,sw+10))/3;
            d7=abs(d71-d72);

            //fprintf(M7clcr,"%d\n",d7);

            if(key!=1 && d7<=50)
            {
                key=1;
                //printf("error:%d , Alignment OK!\n",d7);

                vel_L=1000;
                vel_R=1000;
                bodyJOGvel(vel_L,vel_R);
                //bodyJOGvel(1000,1000); //650,650
                usleep(50000);
//                bodyact("fwd");
//                usleep(30000);
            }
            else if(key!=2 && d7>=51 && d71>d72)
            {
                key=2;
                //printf("error:%d , need some left\n",d7);

                vel_L=850;
                vel_R=1000;
                bodyJOGvel(vel_L,vel_R);
                //bodyJOGvel(850,1000); //250,250
                usleep(50000);//0.2

//                bodyact("left");
//                usleep(300000);
//                bodyact("stop");
//                usleep(300000);
            }
            else if(key!=3 && d7>=51 && d71<d72)
            {
                key=3;
                printf("error:%d , need some right\n",d7);

                vel_L=1000;
                vel_R=850;
                bodyJOGvel(vel_L,vel_R);
                //bodyJOGvel(1000,850);
                usleep(50000);

//                bodyact("right");
//                usleep(300000);
//                bodyact("stop");
//                usleep(300000);
            }

            streamDepth.readFrame( &frameDepth );
            d=depthMD(320,80);
        }
        else
        {
				printf("mode_7 : Stairs End\n");
				//bodyJOGvel(400,400);
				//usleep(50000));
				//bodyact("fwd");
				//usleep(250000);

                //legact("stop","stop");
                //usleep(30000);

				streamDepth.readFrame( &frameDepth );
				d=depthMD(320,80);


        }
        //printf("d = %d \n",d);
        //fprintf(M7VL,"%d\n",vel_L);
        //fprintf(M7VR,"%d\n",vel_R);
        //fprintf(M7DD,"%d\n",d);
    }

	//----down stairs end , prepare-----
	printf("Landind prepare...\n");

	bodyact("stop");
	usleep(50000);

	/*---Motor Set---*/

	//fprintf(M7VL,"Down stairs end prepare = %d\n",350);
    //fprintf(M7VR,"Down stairs end prepare = %d\n",350);
	bodyJOGvel(350,350);
	usleep(50000);

	legMAvel(50,50); //80,50
	usleep(50000);
	legcs(0,0);
	usleep(50000);

	bodyact("fwd");
	usleep(2800000);

	bodyact("stop");
	usleep(50000);

	legma(0,-27000); //T3 up
	usleep(50000); //0.2

	bodyact("fwd");
	usleep(3500000);

	legma(-28000,-27000);
	usleep(3000000);

	bodyact("fwd");
	usleep(1000000);

//	bodyact("stop");
//	usleep(50000);

	XtionMotor(0,0);
	usleep(300000);

	legMAvel(1000,500); //new 6/1
	usleep(1000000); //new 6/1

	bodyact("fwd");
	usleep(2000000);
    //legMAvel(200,200);
	//usleep(2000000);

	bodyact("stop");
	usleep(500000);


	legMAvel(1000,1000); //new 6/1
    usleep(500000); //new 6/1

    legma(-29300,-28500);
    //legma(-28000,-27000);
	usleep(500000);
	sleep(3);
	//Check_MotorPE(1,3);


	//action_4f(); //p1

    /*---Time end---*/
//    time_t t2 = time(NULL);
    //fprintf(M7XD,"Mode7 Time = %d\n",t2-t1);

    /*---Data Record - Close---*/
//    fclose(M7XD);
//    fclose(M7PL);
//    fclose(M7VL);
//    fclose(M7VR);
//    fclose(M7clcr);
//    fclose(M7DD);

	printf("Down stairs END ! \n");

	//*out_mode=1;//set for continue 0 or 10.

}//mode7 end
