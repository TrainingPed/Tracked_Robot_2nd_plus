#include <string.h>
/*---Tracked Robot Manual Control---*/

int TrackRobot_ManualControl();
void Data_test_mode9();
void Data_test_Climb();


int vv1=1000,vv2=1000,y=0;
int wd1=0,wd2=0,wd3=0;
int act;
//----------------
//1=fwd
//2=back
//3=left
//4=right
//0=stop
//
//
//
//
//----------------
int TrackRobot_ManualControl()
{
    printf("Manual control \n");

    while(1)
    {
        y=getch();

        if(y==32){
            bodyact("stop");
            //XtionMotor_stop();
            //legact("stop","stop");
            //usleep(50000);
			act = 0;
        }
        else if(y==65){
            bodyact("fwd");
            //usleep(50000);
			act = 1;
        }
        else if(y==66){
            bodyact("back");
            //usleep(50000);
			act = 2;
        }
        else if(y==68){
            bodyact("left");
            //usleep(50000);
			act = 3;
        }
        else if(y==67){
            bodyact("right");
            //usleep(50000);
			act = 4;
        }
        else if(y==97||y==65)//A
            //legma(0,5000);
            legact("","rev");
        else if(y==122||y==90)//Z
            //legma(0,-5000);
            legact("","fwd");
        else if(y==83||y==115)//S
            //legma(5000,0);
            legact("rev","");
        else if(y==120||y==88)//X
            //legma(-5000,0);
            legact("fwd","");
        //else if(y==97||y==65)//test
        //{
        //    bodymr(500,500);
        //}
        else if(y==114||y==82)//R reset speed
        {
            bodyJOGvel(300,300);
            legJOGvel(100,100);
            vv1=350;
            vv2=350;
        }
        /*else if(y==112)
        {
            XtionMotor_MR(1000,1000);
        }
        else if(y==111)
        {
            XtionMotor_MR(-1000,-1000);
        }*/
        else if(y==102||y==70)//F fast speed
        {
            bodyJOGvel(1000,1000);
            legJOGvel(200,200);
            vv1=1000;
            vv2=1000;
        }
        else if(y==85||y==117)//U add speed
        {
            if(vv1<30000)
                vv1+=500;
            if(vv2<30000)
                vv2+=500;
            bodyJOGvel(vv1,vv2);

			if (act == 1)
				bodyact("fwd");
			else if (act == 2)
				bodyact("back");
			else if (act == 3)
				bodyact("left");
			else if (act == 4)
				bodyact("right");
			else if (act == 0)
				bodyact("stop");

        }
		else if (y == 121)//y slow down
		{
			if (vv1>1000)
				vv1 -= 500;
			if (vv2>1000)
				vv2 -= 500;
			bodyJOGvel(vv1, vv2);

			if (act == 1)
				bodyact("fwd");
			else if (act == 2)
				bodyact("back");
			else if (act == 3)
				bodyact("left");
			else if (act == 4)
				bodyact("right");
			else if (act == 0)
				bodyact("stop");

		}

        else if(y==101||y==69)//E end for break
        {
            bodyact("stop");
            //legact("stop","stop");
            break;
        }
        else if(y==106)// J
        {
            legma(-26000,-27000); //climb stairs legs initial

        }

        printf("speed_left = %d , speed_right = %d \n",vv1,vv2);
		usleep(5000);
		/*---For data get---*/
		//Data_test_Climb();
		//Data_test_mode9();
    }

	return 1;
}
/*
void Data_test_mode9()
{
	int counter=0,counter_L=0,counter_R=0;
    int counter_abs=50;

	streamDepth.readFrame( &frameDepth );

	for(int i = 0 ; i<frameDepth.getHeight()/2;i++){
		counter_L += depthMD(350,i); //mid=320
		counter_R += depthMD(290,i);
	}
	counter_L = (int)(counter_L/(frameDepth.getHeight()/2));
	counter_R = (int)(counter_R/(frameDepth.getHeight()/2));
	counter_abs=abs(counter_L-counter_R);

	for(int i = 0 ; i<frameDepth.getWidth();i++){
		counter += depthMD(i,300);
	}
	counter = (int)(counter/frameDepth.getWidth());

	printf("Aligment : counter_L = %d , counter_R = %d , counter_abs=%d \n",counter_L,counter_R,counter_abs);

}

void Data_test_Climb()
{
    int counter=0,counter_L=0,counter_R=0;
    int counter_abs=50;

	streamDepth.readFrame( &frameDepth );

	for(int i = 160 ; i<=320 ; i++)
	{
		counter_R += depthMD(i,300);
	}
	for(int i = 320 ; i<=480 ; i++)
	{
		counter_L += depthMD(i,300);
	}

	counter_L = (int)(counter_L/160);
	counter_R = (int)(counter_R/160);
	counter_abs=abs(counter_L-counter_R);

	printf("Aligment : counter_L = %d , counter_R = %d , counter_abs=%d \n",counter_L,counter_R,counter_abs);
}*/
