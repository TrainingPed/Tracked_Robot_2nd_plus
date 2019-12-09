/*---Up Stairs Program---*/
float angg=30; //34
float ang=0;

/*---Mode---*/
void mode_1();	/*---find & near stairs---*/
int mode_21();	/*---1m check & Calculated the tilt angle---*/
void mode_2();	/*---Alignment mode---*/
void mode_3();	/*---up stairs prepare---*/
void mode_4();	/*---Climbing mode---*/

/*---Mode21 left or right stairs distance too small need correct---*/
void exceptionLeft();
void exceptionRight();


void mode_1()	//---find & near stairs
{
	/*---Time---*/
    //time_t t1 = time(NULL);
	/*---"depth_find" Function Variable---*/
	int i=0,j=0,x=0,y=0,mid_x=0,mid_y=0,sw,mid_x2=0,mid_y2=0;
	/*---For Fuzzy Variable---*/
	int cont1=0,cont11=0,cont12=0;
	/*---"Fuzzy" Function Motor speed Variable---*/
	double output_r,output_l;
	/*---Depth Data Variable---*/
	int pd=1000,pd2=1000,pd3=1000;
	/*---Patrol Mode : Random---*/
	int rd=0;
	/*---Patrol Mode : Motor speed---*/
	int tr=0,tl=0;
	/*---Patrol Mode & Fuzzy Speed : Luck---*/
    int key=0;
	/*---Out While---*/
	int d=1500;

    /*---Data Record - Open---*/
//    FILE *M1XD=fopen("/home/ubuntu/CodeBlocks/TrackedRobot_Control/Recore Data/M1_XtionDistance.txt","w");
//    FILE *M1PL=fopen("/home/ubuntu/CodeBlocks/TrackedRobot_Control/Recore Data/M1_PixelLocation.txt","w");
//    FILE *M1VL=fopen("/home/ubuntu/CodeBlocks/TrackedRobot_Control/Recore Data/M1_VelLeft.txt","w");
//    FILE *M1VR=fopen("/home/ubuntu/CodeBlocks/TrackedRobot_Control/Recore Data/M1_VelRight.txt","w");


    printf("Mode 1 , Exploration Mode\n");
    /*---Fuzzy just control vel---*/
//    bodyact("fwd");
//    usleep(50000);
    bodyJOGvel(1300,1300);
	usleep(10000);

    while(d>1150)
    {
        streamDepth.readFrame( &frameDepth );
        depth_find(1,&mid_x2,&mid_y2,&mid_x,&mid_y,&i,&j,&x,&sw);

        legact("stop","stop");
        usleep(50000);

        if(mid_x2==0&&mid_y2==0)    //Patrol Mode
        {
            printf("Can't Find Stairs! ''Patrol Mode'' \n");
            bodyJOGvel(1000,1000);
            pd=(depthMD(190,120)+depthMD(190,240)+depthMD(190,360))/3;
            pd2=(depthMD(352,120)+depthMD(352,240)+depthMD(352,360))/3;
            pd3=(depthMD(515,120)+depthMD(515,240)+depthMD(515,360))/3;

            if(key!=1 && (pd<750 || pd2<750 || pd3<750))
            {
                //printf("Obstacle Distance: %d\n",pd);
                key=1;
                bodyact("stop");
                usleep(50000);
                bodyact("right");
                usleep(50000);
                //rd=rand()%180; //give random angle
                //tr=(rd-90)*1053;
                //tl=(rd-90)*1053;
                //printf("random angle=%d\n",rd);
                //bodyma(tl,-tr);
                //usleep(abs(rd-90)*60000);
                //bodycs(0,0);
            }
//            else if(pd2<750)
//            {
//                printf("Obstacle Distance: %d\n",pd2);
//                bodyact("stop");
//                usleep(50000);
//                rd=rand()%180; //give random angle
//                tr=(rd-90)*1053;
//                tl=(rd-90)*1053;
//                //printf("random angle=%d\n",rd);
//                bodyma(tl,-tr);
//                usleep(abs(rd-90)*60000);
//                bodycs(0,0);
//            }
//            else if(pd3<750)
//            {
//                printf("Obstacle Distance: %d\n",pd3);
//                bodyact("stop");
//                usleep(50000);
//                rd=rand()%180; //give random angle
//                tr=(rd-90)*1053;
//                tl=(rd-90)*1053;
//                //printf("random angle=%d\n",rd);
//                bodyma(tl,-tr);
//                usleep(abs(rd-90)*60000);
//                bodycs(0,0);
//            }
            else
            {
                if(key!=2)
                {
                    key=2;
                    bodyact("stop");
                    usleep(50000);
                    bodyact("fwd");
                    usleep(50000);
                }
            }
        }
        else //Fuzzy
        {
            /*---Fuzzy just control vel---*/
            if(key!=3)
            {
                key=3;
                bodyact("stop");
                usleep(50000);
                bodyact("fwd");
                usleep(50000);
            }

            /*---Fuzzy Choose---*/
            cont1++;
            while(cont11<1)
            {
                if(mid_x2<280) cont12=1;
                else if(280<mid_x2<420) cont12=2;
                else if(mid_x2<420) cont12=3;
                cont11++;
            }

            /*----GO FUZZY---*/
            printf("cont12=%d\n",cont12);
            if(cont12==1)
            {
                d=depthMD(mid_x2,mid_y2);

                Fuzzy(d,(mid_x2-30),&output_l,&output_r);

                bodyJOGvel(output_l+500,output_r+500);

//                fprintf(M1XD,"%d\n",d);
//                fprintf(M1PL,"%d\n",mid_x2);
//                fprintf(M1VL,"%f\n",output_l);
//                fprintf(M1VR,"%f\n",output_r);
            }

            else if(cont12==2)
            {
                d=depthMD(mid_x2,mid_y2);

                Fuzzy(d,mid_x2,&output_l,&output_r);

                bodyJOGvel(output_l+500,output_r+500);

//                fprintf(M1XD,"%d\n",d);
//                fprintf(M1PL,"%d\n",mid_x2);
//                fprintf(M1VL,"%f\n",output_l);
//                fprintf(M1VR,"%f\n",output_r);
            }

            else if(cont12==3)
            {
                d=depthMD(mid_x2,mid_y2);

                Fuzzy(d,(mid_x2+50),&output_l,&output_r);

                bodyJOGvel(output_l+500,output_r+500);

//                fprintf(M1XD,"%d\n",d);
//                fprintf(M1PL,"%d\n",mid_x2);
//                fprintf(M1VL,"%f\n",output_l);
//                fprintf(M1VR,"%f\n",output_r);
            }

        }

        if(cont1<5)//2
        {
            d=1500;
        }

    } //while end

    bodyact("stop");
    usleep(5000);

    /*---Time end---*/
//    time_t t2 = time(NULL);
//    fprintf(M1XD,"Mode1 Time = %d\n",t2-t1);

    /*---Data Record - Close---*/
//    fclose(M1XD);
//    fclose(M1PL);
//    fclose(M1VL);
//    fclose(M1VR);

}//mode1 end

int mode_21()	//---Alignment & Calculated the tilt angle
{
    /*---Time---*/
    //time_t t1 = time(NULL);
	/*---"depth_find" Function Variable---*/
	int i=0,j=0,x=0,y=0,mid_x=0,mid_y=0,sw,mid_x2=0,mid_y2=0;
	/*---For Alignment(Mode 21) Luck---*/
    int key=0;//Left & Right
	int key1=0;//Far & Close
	/*---For 1m Alignment---*/
	int counter21 = 10000;

	/*---Depth Data Variable---*/
	int d21=0,cont21=0,i3=0,j3=0,y3=0,sum3=0;
	double xx3=0,t21=0,td21=0;
	float z3=0,x3=0;

	/*---Out While---*/
	int d=800;

	/*---Vel - For Record---*/
	int vel_L=0,vel_R=0;

    /*---Data Record - Open---*/
//    FILE *M2XD=fopen("/home/ubuntu/CodeBlocks/TrackedRobot_Control/Recore Data/M2_XtionDistance.txt","w");
//    FILE *M2PL=fopen("/home/ubuntu/CodeBlocks/TrackedRobot_Control/Recore Data/M2_PixelLocation.txt","w");
//    FILE *M2VL=fopen("/home/ubuntu/CodeBlocks/TrackedRobot_Control/Recore Data/M2_VelLeft.txt","w");
//    FILE *M2VR=fopen("/home/ubuntu/CodeBlocks/TrackedRobot_Control/Recore Data/M2_VelRight.txt","w");
//    FILE *M2dldr=fopen("/home/ubuntu/CodeBlocks/TrackedRobot_Control/Recore Data/M2_dldr.txt","w");

    printf("Mode21,1m Alignment Mode\n");

	while(d>600)
	{
		mid_x2=0;mid_y2=0;

		streamDepth.readFrame( &frameDepth );
		depth_find(21,&mid_x2,&mid_y2,&mid_x,&mid_y,&i,&j,&x,&sw);

		//Judge stairs distance
		d21=(abs((depthMD(mid_x2-100,mid_y2+25)+depthMD(mid_x2-100,mid_y2+30)+depthMD(mid_x2-100,mid_y2+20))/3-(depthMD(mid_x2+100,mid_y2+25)+depthMD(mid_x2+100,mid_y2+30)+depthMD(mid_x2+100,mid_y2+20))/3));
        //fprintf(M2dldr,"%d\n",d21);

		if(d21<=20)
		{
//            fprintf(M2XD,"%d\n",depthMD(mid_x2,mid_y2));
//            fprintf(M2PL,"%d\n",mid_x2);

			for(int i = 0 ; i<frameDepth.getWidth();i++){
				counter21 += depthMD(i,200);
			}
			counter21 = (int)(counter21/frameDepth.getWidth());
			//printf("counter21 = %d\n",counter21);
			usleep(10000);

			if(counter21<=617) //5F:830, 4F:680
			{
				key1=3;
			}

			if(key1!=1 && counter21>617)	//5F:820, 4F:670 ; too far ,TR fwd org=1100  depthMD(mid_x2,mid_y2)>900
			{
				key1=1;
				printf("Too Far\n");
				bodyact("stop");
				usleep(5000);

				vel_L=350;
				vel_R=350;
				bodyJOGvel(vel_L,vel_R);//400

				usleep(5000);
				bodyact("fwd");
				usleep(50000);
			}
			else if(key1!=2 && counter21<550) //5F:780, 4F:590 ; too close,TR back //900 depthMD(mid_x2,mid_y2)<800
			{
				key1=2;
				printf("Too Close\n");
				bodyact("stop");
				usleep(5000);

				vel_L=350;
				vel_R=350;
				bodyJOGvel(vel_L,vel_R);//300

				bodyact("back");
				usleep(50000);
			}
			else if(key1==3)
			{
				printf("d21:%d , Mode2 Alignment OK\n",d21);
				bodyact("stop");
				usleep(5000);

				for(i3=1;i3<480;i3++)
				{
					if((abs(depthMD(320,i3-1)-depthMD(320,i3))>200)&&(abs(depthMD(320,i3-1)-depthMD(320,i3))<350)&&(depthMD(320,i3-1)!=0)&&(depthMD(320,i3)!=0))
					{
						y3=(abs(depthMD(320,i3-1)-depthMD(320,i3)))/10;
						sum3+=y3;
						j3++;
					}
				}

				if(sum3!=0)
				{
					printf("Calculate the angle\n");
					z3=float(sum3/j3);
					xx3=((((21.2/119)-(21.2/123))/(105.5-101.1))*((depthMD(mid_x2,mid_y2)/10)-101.1)+(21.2/123));
					x3=abs(mid_y-mid_y2)*xx3;

					//angg=34;
					//angg=(57.2958*atan(x3,z3)*6);  //org -> ang=57.2958*atan(x3,z3);
					//ang=(float)(57.2958*atan(x3,z3)*6);
					printf("ang=%f \n",angg);


                    /*---Decide to know too close left or right---*/
                    t21=abs(i-mid_x2);
                    td21=t21*xx3;
                    printf(" Left side stairs(td21)= %f cm\n",td21);
                    printf(" Left side stairs(mid_x2)= %d cm\n",mid_x2);


                    if(mid_x2<=300) //330
                    {
                        t21=abs(i-mid_x2);
                        td21=t21*xx3;
                        printf(" Left side stairs(td21)= %f cm\n",td21);
                        return 1;//mode_21 failed because of the left side stairs.
                    }
//                    else if(mid_x2>=331)
//                    {
//                        t21=abs(mid_x2-x);
//                        td21=t21*xx3;
//                        printf("right side stairs(td21)= %f cm\n",td21);
//                        return 2;//mode_21 failed because of the right side stairs.
//                    }

                    /*---Decide to End mode21---*/
					if(td21>50 || angg>45 || angg>20) //lsd (left stairs distance)
					{
						return 0; //mode_21 succeed.
					}
					else return 1;
				}//if(sum3!=0) end
			} //Key3 end
		}//if(d21<=20) end
		else if(key!=1 && d21>=20 && depthMD(mid_x2-100,mid_y2+20)>depthMD(mid_x2+100,mid_y2+20) && depthMD(mid_x2-100,mid_y2+25)>depthMD(mid_x2+100,mid_y2+25))
		{
			printf("d21:%d , Mode2 Need Left\n",d21);
			key=1;
			bodyact("stop");
			usleep(5000);

			vel_L=300;
            vel_R=300;
            bodyJOGvel(vel_L,vel_R);

			bodyact("left");
			usleep(5000);

//				if(depthMD(mid_x2,mid_y2)<930)
//				{
//					bodyJOGvel(500,500);
//					bodyact("back");
//					//usleep(500000); //new
//				}
		}
		else if(key!=2 && d21>=20&&depthMD(mid_x2-100,mid_y2+20)<depthMD(mid_x2+100,mid_y2+20) && depthMD(mid_x2-100,mid_y2+25)<depthMD(mid_x2+100,mid_y2+25))
		{
			printf("d21:%d , Mode2 Need Right\n",d21);
			key=2;
			bodyact("stop");
			usleep(5000);

			vel_L=300;
            vel_R=300;
            bodyJOGvel(vel_L,vel_R);

			bodyact("right");
			usleep(5000);

//				if(depthMD(mid_x2,mid_y2)<930)
//				{
//					bodyJOGvel(500,500);
//					bodyact("back");
//					//usleep(100000); //new
//				}
		}
			streamDepth.readFrame( &frameDepth );
			d=depthMD(mid_x2,mid_y2);
			printf("d=%d,mid_x2=%d,mid_y2=%d\n",d,mid_x2,mid_y2);

//		fprintf(M2VL,"%d\n",vel_L);
//        fprintf(M2VR,"%d\n",vel_R);

	}//while(d>600) end

    /*---Time end---*/
//    time_t t2 = time(NULL);
//    fprintf(M2XD,"Mode2 Time = %d\n",t2-t1);

    /*---Data Record - Close---*/
//    fclose(M2XD);
//    fclose(M2PL);
//    fclose(M2VL);
//    fclose(M2VR);
//    fclose(M2dldr);

}//mode21 end

void mode_3()	//---up stairs prepare
{
	/*---Time---*/
    //time_t t1 = time(NULL);

	/*---Data Record - open---*/
    //FILE *M3Time=fopen("/home/ubuntu/CodeBlocks/TrackedRobot_Control/Recore Data/M3_time.txt","w");

	bodyact("stop");
	XtionMotor(740-angg,0); //375 5F 640

	printf("Mode 3 , Up Stairs Prepare\n");

	if(angg<45 && angg>20)
	{
		/*---Speed set---*/
		bodyJOGvel(450,450);//350,350
		usleep(30000);
		legMAvel(200,200);
		usleep(30000);
		legcs(0,0);

		/*---01---*/
		legma((90-angg)*300,(90-angg)*300); //T0, T3 move down
		sleep(3); //6.5 sec

		/*---02 Time: 7sec---*/
		bodyact("fwd");
		usleep(1000000);
		legma((90-angg)*300,90*300); //T3 move down fist, T0 not move
		usleep(6800000);

		/*---03---*/
		legma(98*300,95*300); //T0 move down, T3 not move
		usleep(6500000);
		bodyact("stop");
		usleep(50000);

		//*out_mode=4;
		printf("mode3 End\n");
	}
	else
	{
		printf("Mode3 Stair Angle Greater or less than Limit\n");
		//mode_21();
		//d=800;
	}

	/*---Time end---*/
//    time_t t2 = time(NULL);
//    fprintf(M3Time,"Mode3 Time = %d\n",t2-t1);

    /*---Data Record - close---*/
//    fclose(M3Time);

}//mode3 end

void mode_4()	//---Climbing mode
{
	/*---Time---*/
//    time_t t1 = time(NULL);

	/*---"depth_find" Function Variable---*/
	int i=0,j=0,x=0,y=0,mid_x=0,mid_y=0,sw,mid_x2=0,mid_y2=0;
	/*---Depth Data Variable---*/
	int d4=0,d41=0,d42=0,cont4=0;
	/*---For Climbing mode(Mode 4) Luck---*/
	int key=0;

	int angle = (int)angg;

    /*---Vel - For Record---*/
	int vel_L=0,vel_R=0;

	/*---Data Record - Open---*/
//    FILE *M4XD=fopen("/home/ubuntu/CodeBlocks/TrackedRobot_Control/Recore Data/M4_XtionDistance.txt","w");
//    FILE *M4PL=fopen("/home/ubuntu/CodeBlocks/TrackedRobot_Control/Recore Data/M4_PixelLocation.txt","w");
//    FILE *M4VL=fopen("/home/ubuntu/CodeBlocks/TrackedRobot_Control/Recore Data/M4_VelLeft.txt","w");
//    FILE *M4VR=fopen("/home/ubuntu/CodeBlocks/TrackedRobot_Control/Recore Data/M4_VelRight.txt","w");
//    FILE *M4clcr=fopen("/home/ubuntu/CodeBlocks/TrackedRobot_Control/Recore Data/M4_clcr.txt","w");
//    FILE *M4tp=fopen("/home/ubuntu/CodeBlocks/TrackedRobot_Control/Recore Data/M4_TopDistance.txt","w");

    printf("Mode 4 , Climbing Mode\n");

    /*---For Murphy's law ---*/
    legMAvel(1,1);
    legJOGvel(1,1);
    legva(50,50);

    bodyact("fwd");
    usleep(50000);

	while(cont4<3)
	{
		mid_x=0;mid_y=0;

		streamDepth.readFrame( &frameDepth );
		depth_find(4,&mid_x2,&mid_y2,&mid_x,&mid_y,&i,&j,&x,&sw);

		//legact("stop","stop");
		//usleep(50000);

		if(mid_x!=0&&mid_y!=0)
		{
			cont4=0;
			d41=(depthMD(x+20,j+30)+depthMD(x+20,j+20)+depthMD(x+20,j+40))/3;
			d42=(depthMD(i-20,sw+30)+depthMD(i-20,sw+20)+depthMD(i-20,sw+40))/3;
            d4=abs(d41-d42);

//            fprintf(M4XD,"%d\n",depthMD(mid_x,mid_y));
//            fprintf(M4PL,"%d\n",mid_x);
//            fprintf(M4clcr,"%d\n",d4);


			//printf("mode4 d4=%d\n",d4);

			if(key!=1 && d4<=55) //40
			{
				key=1;
				//printf("d4=%d ,d41=%d ,d42=%d , Alignment OK!\n",d4,d41,d42);
				vel_L=1000;
                vel_R=1000;
                bodyJOGvel(vel_L,vel_R);
				//bodyJOGvel(1000,1000); //650,650, //800,800
				//bodyact("fwd");
				usleep(50000);
			}
			else if(key!=2 && d4>=56 && d41>d42)
			{
				key=2;
				//printf("d4=%d ,d41=%d ,d42=%d , need some left\n",d4,d41,d42);
//				bodyact("stop");
//				usleep(50000);//0.2
                vel_L=880;
                vel_R=1000;
                bodyJOGvel(vel_L,vel_R);
				//bodyJOGvel(880,1000); //250, //650,800
				usleep(50000);

//				bodyact("left");
//				usleep(300000);//0.3
//				bodyact("stop");
//				usleep(300000);//0.3
			}
			else if(key!=3 && d4>=56 && d41<d42)
			{
				key=3;
				//printf("d4=%d ,d41=%d ,d42=%d , need some right\n",d4,d41,d42);
//				bodyact("stop");
//				usleep(5000);//0.2
                vel_L=1000;
                vel_R=880;
                bodyJOGvel(vel_L,vel_R);
				//bodyJOGvel(1000,880); //250, //800,650
				usleep(50000);//0.2

//				bodyact("right");
//				usleep(300000);//org:0.3
//				bodyact("stop");
//				usleep(300000);
			}

		}//if(mid_x!=0&&mid_y!=0)
		else
		{
			printf("Ready go to the top!\n");
			key=0;
			bodyact("stop");
			usleep(50000);

            vel_L=400;
            vel_R=400;
            bodyJOGvel(vel_L,vel_R);
            //bodyJOGvel(400,400); //for cont4 check dont stop
            bodyact("fwd");
            usleep(50000);//0.5
			cont4++;
		}

//        fprintf(M4VL,"%d\n",vel_L);
//        fprintf(M4VR,"%d\n",vel_R);

	}//while(cont4<3) end

	printf("---Near to TOP---\n\n\n");
	/*---ALL STOP---*/
    bodyact("stop");
    legact("stop","stop");
    usleep(10000);

    legva(600,600);
	usleep(10000);

    //sleep(5);
	/*---01---*/
//    fprintf(M4VL,"Near to top = %d\n",700);
//    fprintf(M4VR,"Near to top = %d\n",700);
	bodyJOGvel_select(1,700);
	bodyJOGvel_select(2,700);
	usleep(10000);


	//bodyJOGvel(450,450); //400,400
	bodyact("fwd");
	usleep(6300000); //org:6500000 ,5700000

	printf("---TOP Lay Down Front Arm---\n\n\n");

	/*---02.Stop and set speed---*/
	bodyact("stop");
	usleep(50000);//0.5
//	fprintf(M4VL,"%d\n",400);
//    fprintf(M4VR,"%d\n",400);
	bodyJOGvel(400,400);
	legMAvel(200,200);
	XtionMotor(0,0);
    usleep(10000);//new 6/1

	/*---03. Time : 7sec---*/
	legma(95*300,135*300);
	usleep(3000000);
	bodyact("fwd");
	usleep(4000000);

	/*---04---*/
	legma(0,0);
	usleep(2000000);

	legMAvel(800,800);
	usleep(50000);

	bodyact("stop");
    usleep(50000);

	/*---Time end---*/
//    time_t t2 = time(NULL);
//    fprintf(M4XD,"Mode4 Time = %d\n",t2-t1);


    /*---Data Record - Close---*/
//    fclose(M4XD);
//    fclose(M4PL);
//    fclose(M4VL);
//    fclose(M4VR);

	printf("---Already On the TOP---\n");
	printf("Mode4 End \n\n\n");

}//mode4 end

/*---Stairs distance correction---*/
void exceptionLeft()
{
    exceptionRight();
    printf("Too close Left, go into the exception mode!!!!!");
}

void exceptionRight()
{
    bodyJOGvel(500,500);
    usleep(50000);

    bodyact("left");
    usleep(1000000);

    bodyact("back");
    usleep(4000000);

    bodyact("stop");
    usleep(50000);
}


/*---Not use---*/

void mode_2()	//---Alignment mode again
{
	/*---"depth_find" Function Variable---*/
	int i=0,j=0,x=0,y=0,mid_x=0,mid_y=0,sw,mid_x2=0,mid_y2=0;
	/*---Depth Data Variable---*/
	int d2=0;
	/*---For Alignment(Mode 2) Luck---*/
    int key=0;//Left & Right
	/*---Out While---*/
	int d=1500;

	printf("Mode 2 , Alignment Mode\n");


	bodyact("stop");

	while(d>850)
	{
		streamDepth.readFrame( &frameDepth );
		depth_find(2,&mid_x2,&mid_y2,&mid_x,&mid_y,&i,&j,&x,&sw);

		d2=(abs((depthMD(x,j+10)+depthMD(x,j+20)+depthMD(x,j+30))/3-(depthMD(i,sw+10)+depthMD(i,sw+20)+depthMD(i,sw+30))/3));
		printf("org_d2=%d \n",d2);

		if(d2<=20)
		{
			printf("d2=%d , Mode21 Alignment OK!\n",d2);
			//bodyact("stop");
			bodyJOGvel(350,350);
			bodyact("fwd");
			usleep(300000);//0.3
			//*out_mode=3;
		}
		else if(key!=1 && d2>=20&&depthMD(x,j+20)>depthMD(i,sw+20)&&depthMD(x,j+10)>depthMD(i,sw+10))
		{
			printf("d2=%d Mode21 Left\n",d2);
			key=1;
			bodyact("stop");
			usleep(50000);//0.2
			bodyJOGvel(300,300);
			bodyact("left");
			usleep(200000);
			//bodyact("stop");
			//usleep(50000);//1 sec
		}
		else if(key!=2 && d2>=20&&depthMD(x,j+20)<depthMD(i,sw+20))
		{
			printf("d2=%d Mode21 Right\n",d2);
			key=2;
			bodyact("stop");
			usleep(50000);//0.2
			bodyJOGvel(300,300);
			bodyact("right");
			usleep(200000);
			//bodyact("stop");
			//usleep(50000);//1sec
		}

		streamDepth.readFrame( &frameDepth ); //new
		d=depthMD(mid_x,mid_y+20); //orignal+10

	}//while end

}//mode2 end

