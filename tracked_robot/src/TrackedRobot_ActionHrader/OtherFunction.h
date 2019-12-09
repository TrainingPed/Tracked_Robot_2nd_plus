#define sensorName "/dev/ttymxc3"

//Wall Following Variable
int m9_1_d1=0,m9_1_d2=0,m9_1_d3=0;
int m9_2_d1=0,m9_2_d2=0,m9_2_d3=0;
int ds1_1=0,ds1_2=0,ds1_3=0;
int fd_Sensor;

int getch(void);	/*---c++  getch() function---*/
double atan(double x3,double z3);	/*---Calculate---*/

int sensorData(int sensorNumber);	/*---Get Arduino GPIO Data---*/
int Sensor_serialport_read(int fd,char* buf,char until); /*---Wait Arduino GPIO Data and Return for 'sensorData'---*/
void Sensor_close();

void mode_9(int in_mode);	/*---Function : to link up other mode---*/
void action_4f();	//s1, s2, s3, s4
void action_5f();	//s5



int getch (void)
{
    int ch;
    struct termios oldt,newt;

    tcgetattr(STDIN_FILENO, &oldt);
    memcpy(&newt, &oldt, sizeof(newt));
    newt.c_lflag &= ~(ECHO|ICANON|ECHOE|ECHOK|ECHONL|ECHOPRT|ECHOKE|ICRNL);
    tcsetattr(STDIN_FILENO,TCSANOW,&newt);
    ch = getchar();
    //tcsetattr(STDIN_FILENO,TCSANOW,&oldt);

    return ch;
}

double atan(double x3,double z3)
{
	return(atan(x3/z3));
}

/*---sensor ---*/
int sensorData(int sensorNumber)
{
    //fd=open(sensorName,O_RDWR | O_NOCTTY | O_NDELAY);
    fd_Sensor=open(sensorName,O_RDWR | O_NOCTTY );
    if(fd_Sensor<0)
    {
        printf("*** Open ttymxc3 port error ***\n");
        return -1;
    }

    struct termios options;
    tcgetattr(fd_Sensor, &options);

    options.c_cflag = B9600 | CS8 | CLOCAL | CREAD ;
    options.c_iflag = IGNPAR;
    options.c_oflag = OPOST;
    options.c_lflag = ICANON;

//    cfsetispeed(&options,B9600);
//    cfsetospeed(&options,B9600);

//    options.c_cflag &= ~PARENB;
//    options.c_cflag &= ~CSTOPB;
//    options.c_cflag &= ~CSIZE;
//    options.c_cflag |= CS8;
//    options.c_cflag &= ~CRTSCTS;
//    options.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines

//    options.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl
//    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
//    options.c_oflag &= ~OPOST; // make raw

    options.c_cc[VMIN]  = 0;
    options.c_cc[VTIME] = 20;

    if( tcsetattr(fd_Sensor, TCSANOW, &options) !=0)
    {
        printf("*** Error from sensor tcsetattr ***\n");
        close(fd_Sensor);
        return -1;
    }


    int sensorRead01=0;
    /*int sensorRead02=0;
    int sensorRead03=0;
    int sensorRead04=0;
    int sensorRead05=0;*/

    char buf01[20];
    /*char buf02[20];
    char buf03[20];
    char buf04[20];
    char buf05[20];*/

    sensorRead01=Sensor_serialport_read(fd_Sensor,buf01,';');
    /*sensorRead02=Sensor_serialport_read(fd_Sensor,buf02,',');
    sensorRead03=Sensor_serialport_read(fd_Sensor,buf03,'/');
    sensorRead04=Sensor_serialport_read(fd_Sensor,buf04,':');
    sensorRead05=Sensor_serialport_read(fd_Sensor,buf05,'*');*/
    //printf("size: %d  sensorData: %s \n",sensorRead,buf);
    int i=atoi(buf01); //atoi : convert char to int
    /*int j=atoi(buf02);
    int k=atoi(buf03);
    int m=atoi(buf04);
    int b=atoi(buf05);*/

    if(sensorNumber==1)
    {
      return i;
    }
   /* else if(sensorNumber==2)
    {
      return j;
    }
    else if(sensorNumber==3)
    {
      return k;
    }
    else if(sensorNumber==4)
    {
      return m;
    }
    else if(sensorNumber==5)
    {
      return b;
    }*/

    tcflush(fd_Sensor, TCIOFLUSH); //Flush Buffer data

    return 1;

}

int Sensor_serialport_read(int fd, char* buf, char until)
{
    //printf("!!\n
    tcflush(fd, TCIFLUSH);
    usleep(10000);

    char b[1];
    int i=0;
    do
    {
        //printf("??\n");
        int n= read(fd,b,1);
        if(n==-1) return -1;
        if(n==0)
        {
            usleep(10*1000);
            continue;
        }

        buf[i]=b[0]; i++;

    }while(b[0] != until);

    buf[i]=0;
    return i;
}

void Sensor_close()
{
    close(fd_Sensor);
}

void mode_9(int in_mode)//---------Continuous----------
{
    printf("Mode9, Continuous mode\n");

/*---Mode9-0---*/
	if(in_mode==0) //For Continuous Up stairs (5F to 6F)
	{
        printf("Mode9-0 : Continuous Up stairs\n");
        /*---Step1. First colse---*/
        printf("Step1 : First colse, Go Straight\n");
		/*---Step1 : plan A---*/
		int counter=0,counter_L=0,counter_R=0;
        int counter_abs=50;
        int while_juge=50;
        int key=0,key1=0;

		bodyJOGvel(600,600); //700
		usleep(5000);
		bodyact("fwd");
        usleep(5000);

		while(while_juge>0) //Reference Y-axis (Stairs 4f: 750, 5f:)
		{
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

			/*---Action---*/
            if(counter_abs<=3)
            {
                if(key1!=1 && counter>=550)
                {
                    key1=1;
                    bodyJOGvel(600,600); //600
                    // bodyact("stop");
                    // usleep(5000);
                    // bodyact("fwd");
					usleep(5000);
                }
                if(counter<550)
                {
                    bodyact("stop");
                    usleep(5000);
                    break;
                }
            }
            else if(counter_abs>=4 && counter_abs<=19)
            {
                if(key!=1 && counter_L<counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Right\n",counter_L,counter_R);
                    key=1;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(450,600);
                    //bodyact("right");
                    usleep(5000);
                }
                else if(key!=2 && counter_L>counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Left\n",counter_L,counter_R);
                    key=2;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(600,450);
                    //bodyact("left");
                    usleep(5000);
                }
            }
			else if(counter_abs>=20 && counter_abs<=30)
            {
                if(key!=3 && counter_L<counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Right\n",counter_L,counter_R);
                    key=3;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(350,600);
                    //bodyact("right");
                    usleep(5000);
                }
                else if(key!=4 && counter_L>counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Left\n",counter_L,counter_R);
                    key=4;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(600,350);
                    //bodyact("left");
                    usleep(5000);
                }
            }
			else if(counter_abs>=31)
            {
                if(key!=5 && counter_L<counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Right\n",counter_L,counter_R);
                    key=5;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(250,600);
                    //bodyact("right");
                    usleep(5000);
                }
                else if(key!=6 && counter_L>counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Left\n",counter_L,counter_R);
                    key=6;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(600,250);
                    //bodyact("left");
                    usleep(5000);
                }
            }
		}


		/*---Step1 : plan B---*/
		// int counter = 1200;
        // bodyact("fwd");
        // usleep(50000);
		// while(counter>700) //Reference X-axis (Stairs 4f: 1100, 5f:)
		// {
		    // //counter = 0;
            // streamDepth.readFrame( &frameDepth );
            // for(int i = 0 ; i<frameDepth.getWidth();i++){
                // counter += depthMD(i,200);
            // }
            // counter = (int)(counter/frameDepth.getWidth());
            // printf("Step1 : counter = %d\n", counter);
		// }
        // bodyact("stop");
		// usleep(50000);
		/*---Step1. END---*/

        /*---Step2. Left 90 angle---*/
		printf("Step2 : Left 90 angle\n");
        bodycs(0,0);
		usleep(5000);
        bodyMAvel(700,700);
		usleep(5000);
		bodyma(-87500,88000); //90 degree : 87000 (big number to avoid xtion sceen right side)
		//Check_MotorPE(1,2);
		/*---Step2. END---*/

		/*---Step3. Go Straight---*/
		printf("Step3 : Go Straight\n");
		/*---Step3 : plan A---*/
		counter=0;counter_L=0;counter_R=0;
        counter_abs=50;
        while_juge=50;
        key=0;key1=0;

		bodyJOGvel(600,600); //700
		usleep(5000);
		bodyact("fwd");
        usleep(5000);

        while(while_juge>0) //Reference Y-axis (Stairs 4f: 750, 5f:)
		{
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

			/*---Action---*/
            if(counter_abs<=3)
            {
                if(key1!=1 && counter>=500)
                {
                    key1=1;
                    bodyJOGvel(900,900); //600
                    // bodyact("stop");
                    // usleep(5000);
                    // bodyact("fwd");
					usleep(5000);
                }
                if(counter<500)
                {
                    bodyact("stop");
                    usleep(5000);
                    break;
                }
            }
            else if(counter_abs>=4 && counter_abs<=19)
            {
                if(key!=1 && counter_L<counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Right\n",counter_L,counter_R);
                    key=1;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(750,900);
                    //bodyact("right");
                    usleep(5000);
                }
                else if(key!=2 && counter_L>counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Left\n",counter_L,counter_R);
                    key=2;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(900,750);
                    //bodyact("left");
                    usleep(5000);
                }
            }
			else if(counter_abs>=20 && counter_abs<=30)
            {
                if(key!=3 && counter_L<counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Right\n",counter_L,counter_R);
                    key=3;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(650,900); //350,600
                    //bodyact("right");
                    usleep(5000);
                }
                else if(key!=4 && counter_L>counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Left\n",counter_L,counter_R);
                    key=4;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(900,650); //600,350
                    //bodyact("left");
                    usleep(5000);
                }
            }
			else if(counter_abs>=31)
            {
                if(key!=5 && counter_L<counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Right\n",counter_L,counter_R);
                    key=5;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(550,900); //250,600
                    //bodyact("right");
                    usleep(5000);
                }
                else if(key!=6 && counter_L>counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Left\n",counter_L,counter_R);
                    key=6;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(900,550); //600.250
                    //bodyact("left");
                    usleep(5000);
                }
            }
		}

		/*---Step3 : plan B---*/
		// bodyJOGvel(700,700);
		// usleep(500000);

        // counter = 900;
        // bodyact("fwd");
        // usleep(50000);
		// while(counter>750) //Reference Y-axis (Stairs 4f: 750, 5f:)
		// {
		    // //counter = 0;
            // streamDepth.readFrame( &frameDepth );
            // for(int i = 0 ; i<frameDepth.getHeight()/2;i++){
                // counter += depthMD(320,i);
            // }
            // counter = (int)(counter/(frameDepth.getHeight()/2));
            // printf("Step3 : counter = %d\n", counter);
		// }
		// bodyact("stop");
		// usleep(50000);
		/*---Step3. END---*/

		/*---Step4. Left 90 angle parallel next stirs---*/
		printf("Step4 : Left 90 angle parallel next stirs\n");
		bodycs(0,0);
		usleep(5000);
        bodyMAvel(700,700);
		usleep(5000);
		bodyma(-87500,87500); //90 degree
        //bodyma(-175500,175500); //180 degree
		//Check_MotorPE(1,2);
		usleep(50000);

		/*---Step4. END---*/

		printf("Mode9-0 end\n");
	}

/*---Mode9-1---*/
	if(in_mode==1) //For Up and Down stairs(turn 180)
	{
		printf("Mode9-1 : Up and Down stairs\n");

        int counter=0,counter_L=0,counter_R=0;
        int counter_abs=50;
        int while_juge=50;
        int key=0,key1=0;

		bodyJOGvel(600,600); //700
		usleep(5000);
		bodyact("fwd");
        usleep(5000);

		while(while_juge>0) //Reference Y-axis (Stairs 4f: 750, 5f:)
		{
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

			/*---Action---*/
            if(counter_abs<=3)
            {
                if(key1!=1 && counter>=1150) //5F: 550, 4F:1550
                {
                    key1=1;
                    bodyJOGvel(600,600); //600
                    // bodyact("stop");
                    // usleep(5000);
                    // bodyact("fwd");
					usleep(5000);
                }
                if(counter<1150) //5F:550 4F:1550
                {
                    bodyact("stop");
                    usleep(5000);
                    break;
                }
            }
            else if(counter_abs>=4 && counter_abs<=19)
            {
                if(key!=1 && counter_L<counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Right\n",counter_L,counter_R);
                    key=1;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(500,600);
                    //bodyact("right");
                    usleep(5000);
                }
                else if(key!=2 && counter_L>counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Left\n",counter_L,counter_R);
                    key=2;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(600,500);
                    //bodyact("left");
                    usleep(5000);
                }
            }
			else if(counter_abs>=20 && counter_abs<=30)
            {
                if(key!=3 && counter_L<counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Right\n",counter_L,counter_R);
                    key=3;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(400,600);
                    //bodyact("right");
                    usleep(5000);
                }
                else if(key!=4 && counter_L>counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Left\n",counter_L,counter_R);
                    key=4;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(600,400);
                    //bodyact("left");
                    usleep(5000);
                }
            }
			else if(counter_abs>=31)
            {
                if(key!=5 && counter_L<counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Right\n",counter_L,counter_R);
                    key=5;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(300,600);
                    //bodyact("right");
                    usleep(5000);
                }
                else if(key!=6 && counter_L>counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Left\n",counter_L,counter_R);
                    key=6;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(600,300);
                    //bodyact("left");
                    usleep(5000);
                }
            }
		}

		/*---180 Degree---*/
		printf("Rotation 180 Degree...\n");

        XtionMotor(500,0); //For Mode5 Prepare
        bodycs(0,0);
        bodyMAvel(700,700);
		//bodyma(-87000,87000); //90 degree
        bodyma(-170000,170000); //180 degree;-175500,175500
		//Check_MotorPE(1,2);
		//sleep(8);
		//bodyact("stop");
		//usleep(50000);
		printf("Mode9-1 end\n");
	}

/*---Mode9-2---*/
	if(in_mode==2) //For Continuous Up stairs (4F to 5F)
	{
        printf("Mode9-2 : Continuous Up stairs tO 5F\n");
        /*---Step1. First colse---*/
        printf("Step1 : First colse, Go Straight\n");
		/*---Step1 : plan A---*/
		int counter=0,counter_L=0,counter_R=0;
        int counter_abs=50;
        int while_juge=50;
        int key=0,key1=0;

		bodyJOGvel(600,600); //700
		usleep(5000);
		bodyact("fwd");
        usleep(5000);

		while(while_juge>0) //Reference Y-axis (Stairs 4f: 750, 5f:)
		{
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

            //printf("Aligment : counter_L = %d , counter_R = %d , counter=%d , counter_abs=%d \n",counter_L,counter_R,counter,counter_abs);

			/*---Action---*/
			if(counter<1350) //distance here
            {
                bodyact("stop");
                usleep(5000);
                break;
            }
            else if(counter_abs<=3)
            {
                if(key1!=1 && counter>=1350) //distance  here
                {
                    key1=1;
                    bodyJOGvel(600,600); //600
                    // bodyact("stop");
                    // usleep(5000);
                    // bodyact("fwd");
					usleep(5000);
                }
//                if(counter<1350) //distance here
//                {
//                    bodyact("stop");
//                    usleep(5000);
//                    break;
//                }
            }
            else if(counter_abs>=4 && counter_abs<=19)
            {
                if(key!=1 && counter_L<counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Right\n",counter_L,counter_R);
                    key=1;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(450,600);
                    //bodyact("right");
                    usleep(5000);
                }
                else if(key!=2 && counter_L>counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Left\n",counter_L,counter_R);
                    key=2;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(600,450);
                    //bodyact("left");
                    usleep(5000);
                }
            }
			else if(counter_abs>=20 && counter_abs<=30)
            {
                if(key!=3 && counter_L<counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Right\n",counter_L,counter_R);
                    key=3;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(350,600);
                    //bodyact("right");
                    usleep(5000);
                }
                else if(key!=4 && counter_L>counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Left\n",counter_L,counter_R);
                    key=4;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(600,350);
                    //bodyact("left");
                    usleep(5000);
                }
            }
			else if(counter_abs>=31)
            {
                if(key!=5 && counter_L<counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Right\n",counter_L,counter_R);
                    key=5;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(250,600);
                    //bodyact("right");
                    usleep(5000);
                }
                else if(key!=6 && counter_L>counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Left\n",counter_L,counter_R);
                    key=6;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(600,250);
                    //bodyact("left");
                    usleep(5000);
                }
            }
		}

        /*---Step2. Left 90 angle---*/
		printf("Step2 : Left 90 angle\n");
        bodycs(0,0);
		usleep(5000);
        bodyMAvel(700,700);
		usleep(5000);
		bodyma(-87500,88000); //90 degree : 87000 (big number to avoid xtion sceen right side)
		//Check_MotorPE(1,2);
		/*---Step2. END---*/

		/*---Step3. Go Straight---*/
		printf("Step3 : Go Straight\n");
		/*---Step3 : plan A---*/
		counter=0;counter_L=0;counter_R=0;
        counter_abs=50;
        while_juge=50;
        key=0;key1=0;

		bodyJOGvel(600,600); //700
		usleep(5000);
		bodyact("fwd");
        usleep(5000);

        while(while_juge>0) //Reference Y-axis (Stairs 4f: 750, 5f:)
		{
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

            //printf("Aligment : counter_L = %d , counter_R = %d , counter_abs=%d \n",counter_L,counter_R,counter_abs);

			/*---Action---*/
            if(counter<500) //distance here
            {
                bodyact("stop");
                usleep(5000);
                break;
            }
            else if(counter_abs<=3)
            {
                if(key1!=1 && counter>=500) //distance here
                {
                    key1=1;
                    bodyJOGvel(900,900); //600
                    // bodyact("stop");
                    // usleep(5000);
                    // bodyact("fwd");
					usleep(5000);
                }

            }
            else if(counter_abs>=4 && counter_abs<=19)
            {
                if(key!=1 && counter_L<counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Right\n",counter_L,counter_R);
                    key=1;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(750,900);
                    //bodyact("right");
                    usleep(5000);
                }
                else if(key!=2 && counter_L>counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Left\n",counter_L,counter_R);
                    key=2;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(900,750);
                    //bodyact("left");
                    usleep(5000);
                }
            }
			else if(counter_abs>=20 && counter_abs<=30)
            {
                if(key!=3 && counter_L<counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Right\n",counter_L,counter_R);
                    key=3;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(650,900); //350,600
                    //bodyact("right");
                    usleep(5000);
                }
                else if(key!=4 && counter_L>counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Left\n",counter_L,counter_R);
                    key=4;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(900,650); //600,350
                    //bodyact("left");
                    usleep(5000);
                }
            }
			else if(counter_abs>=31)
            {
                if(key!=5 && counter_L<counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Right\n",counter_L,counter_R);
                    key=5;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(550,900); //250,600
                    //bodyact("right");
                    usleep(5000);
                }
                else if(key!=6 && counter_L>counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Left\n",counter_L,counter_R);
                    key=6;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(900,550); //600.250
                    //bodyact("left");
                    usleep(5000);
                }
            }
		}

		/*---Step4. Left 90 angle parallel next stirs---*/
		printf("Step4 : Left 90 angle parallel next stirs\n");
		bodycs(0,0);
		usleep(5000);
        bodyMAvel(700,700);
		usleep(5000);
		bodyma(-87500,87500); //90 degree
        //bodyma(-175500,175500); //180 degree
		//Check_MotorPE(1,2);
		usleep(50000);

		/*---Step4. END---*/
		printf("Mode9-2 end\n");
	}

/*---Mode9-3---*/
    if(in_mode==3) //For Continuous Down stairs (5F to 4F)
	{
        printf("Mode9-3 : Continuous Down stairs to 4F\n");
        /*---Step1. First colse---*/
        printf("Step1 : First colse, Go Straight\n");
		/*---Step1 : plan A---*/
		int counter=0,counter_L=0,counter_R=0;
        int counter_abs=50;
        int while_juge=50;
        int key=0,key1=0;

		bodyJOGvel(600,600); //700
		usleep(5000);
		bodyact("fwd");
        usleep(5000);

		while(while_juge>0) //Reference Y-axis (Stairs 4f: 750, 5f:)
		{
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

			/*---Action---*/
            if(counter_abs<=3)
            {
                if(key1!=1 && counter>=1000) //distance  here
                {
                    key1=1;
                    bodyJOGvel(600,600); //600
                    // bodyact("stop");
                    // usleep(5000);
                    // bodyact("fwd");
					usleep(5000);
                }
                if(counter<1000) //distance here
                {
                    bodyact("stop");
                    usleep(5000);
                    break;
                }
            }
            else if(counter_abs>=4 && counter_abs<=19)
            {
                if(key!=1 && counter_L<counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Right\n",counter_L,counter_R);
                    key=1;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(450,600);
                    //bodyact("right");
                    usleep(5000);
                }
                else if(key!=2 && counter_L>counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Left\n",counter_L,counter_R);
                    key=2;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(600,450);
                    //bodyact("left");
                    usleep(5000);
                }
            }
			else if(counter_abs>=20 && counter_abs<=30)
            {
                if(key!=3 && counter_L<counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Right\n",counter_L,counter_R);
                    key=3;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(350,600);
                    //bodyact("right");
                    usleep(5000);
                }
                else if(key!=4 && counter_L>counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Left\n",counter_L,counter_R);
                    key=4;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(600,350);
                    //bodyact("left");
                    usleep(5000);
                }
            }
			else if(counter_abs>=31)
            {
                if(key!=5 && counter_L<counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Right\n",counter_L,counter_R);
                    key=5;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(250,600);
                    //bodyact("right");
                    usleep(5000);
                }
                else if(key!=6 && counter_L>counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Left\n",counter_L,counter_R);
                    key=6;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(600,250);
                    //bodyact("left");
                    usleep(5000);
                }
            }
		}


        sleep(1);
        /*---Step2. Left 90 angle---*/
		printf("Step2 : Right 90 angle\n");
        bodycs(0,0);
		usleep(100000);
        bodyMAvel(700,700);
		usleep(100000);
		//bodyma(86500,-87500);
		bodyma(88000,-87500); //90 degree : 87000 (big number to avoid xtion sceen right side)
		sleep(3);
		//Check_MotorPE(1,2);
		/*---Step2. END---*/

		/*---Step3. Go Straight---*/
        sleep(1);
		printf("Step3 : Go Straight\n");
		/*---Step3 : plan A---*/
		counter=0;counter_L=0;counter_R=0;
        counter_abs=50;
        while_juge=50;
        key=0;key1=0;

		bodyJOGvel(600,600); //700
		usleep(100000);
		bodyact("fwd");
        usleep(100000);

        while(while_juge>0) //Reference Y-axis (Stairs 4f: 750, 5f:)
		{
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

			/*---Action---*/
            if(counter_abs<=3)
            {
                if(key1!=1 && counter>=500) //distance here
                {
                    key1=1;
                    bodyJOGvel(900,900); //600
                    // bodyact("stop");
                    // usleep(5000);
                    // bodyact("fwd");
					usleep(5000);
                }
                if(counter<500) //distance here
                {
                    bodyact("stop");
                    usleep(5000);
                    break;
                }
            }
            else if(counter_abs>=4 && counter_abs<=19)
            {
                if(key!=1 && counter_L<counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Right\n",counter_L,counter_R);
                    key=1;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(750,900);
                    //bodyact("right");
                    usleep(5000);
                }
                else if(key!=2 && counter_L>counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Left\n",counter_L,counter_R);
                    key=2;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(900,750);
                    //bodyact("left");
                    usleep(5000);
                }
            }
			else if(counter_abs>=20 && counter_abs<=30)
            {
                if(key!=3 && counter_L<counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Right\n",counter_L,counter_R);
                    key=3;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(650,900); //350,600
                    //bodyact("right");
                    usleep(5000);
                }
                else if(key!=4 && counter_L>counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Left\n",counter_L,counter_R);
                    key=4;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(900,650); //600,350
                    //bodyact("left");
                    usleep(5000);
                }
            }
			else if(counter_abs>=31)
            {
                if(key!=5 && counter_L<counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Right\n",counter_L,counter_R);
                    key=5;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(550,900); //250,600
                    //bodyact("right");
                    usleep(5000);
                }
                else if(key!=6 && counter_L>counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Left\n",counter_L,counter_R);
                    key=6;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(900,550); //600.250
                    //bodyact("left");
                    usleep(5000);
                }
            }
		}

		/*---Step4. Right 90 angle parallel next stirs---*/
		printf("Step4 : Right 90 angle parallel next stirs\n");
		bodycs(0,0);
		usleep(5000);
        bodyMAvel(700,700);
		usleep(5000);
		bodyma(86500,-87500); //90 degree
        //bodyma(-175500,175500); //180 degree
		//Check_MotorPE(1,2);
		usleep(50000);

//        bodyact("fwd");
//        usleep(1000000);

        bodyact("stop");
        sleep(1);
		/*---Step4. END---*/

		printf("Mode9-3 end\n");
	}

}//mode9 end

void action_4f()
{
    // s1: 4F alignment
    bodyJOGvel(1000,1000);
    sleep(1);

    printf("S1 \n");
//     int counter = 1501;
//		// while(m9_1_d1>800 || m9_1_d2>800 || m9_1_d3>800) //org=700
//		while(counter>1500)
//		{
//
//		    counter = 0;
//            streamDepth.readFrame( &frameDepth );
//            for(int i = 0 ; i<frameDepth.getWidth();i++){
//                counter += depthMD(i,300);
//            }
//            counter = (int)(counter/frameDepth.getWidth());
//            printf("counter = %d\n", counter);
//
//			bodyact("fwd");
//			usleep(300000);
//
//			// printf("Wall distance: %d\n",d);
//		}

        int counter=0,counter_L=0,counter_R=0;
        int counter_abs=50;
        int while_juge=1550;
        int key=0,key1=0;
        int check_xtion_key=0;

		bodyJOGvel(600,600); //700
		usleep(50000);
		bodyact("fwd");
        usleep(50000);

        //printf("w_j=%d \n",while_juge);
        while(while_juge>1500) //Reference Y-axis (Stairs 4f: 750, 5f:)
		{
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

			/*---Action---*/
            if(counter_abs<=3)
            {
                if(key1!=1 && counter>=1600)
                {
                    key1=1;
                    bodyJOGvel(900,900); //600
                    // bodyact("stop");
                    // usleep(5000);
                    // bodyact("fwd");
					usleep(5000);
					check_xtion_key=0;
                }
                if(counter<1600)
                {
                    check_xtion_key++;
                    if(check_xtion_key>3)
                    {
                        bodyact("stop");
                        usleep(5000);
                        break;
                    }
                }
            }
            else if(counter_abs>=4 && counter_abs<=19)
            {
                check_xtion_key=0;
                if(key!=1 && counter_L<counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Right\n",counter_L,counter_R);
                    key=1;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(750,900);
                    //bodyact("right");
                    usleep(5000);

                }
                else if(key!=2 && counter_L>counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Left\n",counter_L,counter_R);
                    key=2;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(900,750);
                    //bodyact("left");
                    usleep(5000);

                }
            }
			else if(counter_abs>=20 && counter_abs<=30)
            {
                check_xtion_key=0;
                if(key!=3 && counter_L<counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Right\n",counter_L,counter_R);
                    key=3;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(650,900); //350,600
                    //bodyact("right");
                    usleep(5000);

                }
                else if(key!=4 && counter_L>counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Left\n",counter_L,counter_R);
                    key=4;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(900,650); //600,350
                    //bodyact("left");
                    usleep(5000);

                }
            }
			else if(counter_abs>=31)
            {
                check_xtion_key=0;
                if(key!=5 && counter_L<counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Right\n",counter_L,counter_R);
                    key=5;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(550,900); //250,600
                    //bodyact("right");
                    usleep(5000);
                }
                else if(key!=6 && counter_L>counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Left\n",counter_L,counter_R);
                    key=6;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(900,550); //600.250
                    //bodyact("left");
                    usleep(5000);
                }
            }
		}

    // s2: go to 4F wall following ready
    printf("S2 \n");
   // bodycs(0,0);
   // usleep(100000);

   printf("Step4 : Left 90 angle parallel next stirs\n");
    bodycs(0,0);
    usleep(5000);
    bodyMAvel(700,700);
    usleep(5000);
    bodyma(87500,-87500);

//    bodyJOGvel(500,500);
//    bodyact("right");
//    usleep(4900000); //9800
//    bodyact("stop");
//    usleep(200000);//right 90 degree

    bodyJOGvel(1000,1000);//go street
    usleep(100000);
    bodyact("fwd");
    sleep(13);
    bodyact("stop");
    usleep(300000);

    //--- Wall Following 01
    bodyJOGvel(1000,1000);
    legMAvel(200,200);
    usleep(100000);

	XtionMotor(0,500); //motor 01
    usleep(2500000);
    WallFollowing_RightWall();

    bodyact("stop");

    XtionMotor(0,0); //motor 01
    //sleep(1);

    //--- s3: 180 degree
    bodycs(0,0);
    bodyMAvel(700,700);
    //bodyma(-87000,87000); //90 degree
    bodyma(-175500,175500); //180 degree
    //Check_MotorPE(1,2);

    /*
    bodyJOGvel(500,500);
    bodyact("right");
    sleep(11); //usleep(9800000)
    bodyact("stop");
    usleep(200000);//right 180 degree
    */

    //--- Wall Following 02

    bodyJOGvel(1000,1000);
    legMAvel(200,200);
    usleep(100000);

	XtionMotor(0,-500); //motor 01
    usleep(2500000);
    WallFollowing_LeftWall();

//--- s4: go to mode1
    printf("WTF s4!!! \n");
    bodyact("fwd");
    usleep(17000000); //15 sec
    bodyact("stop");

    printf("WTF go left \n");
    bodycs(0,0);
    usleep(5000);
    bodyMAvel(700,700);
    usleep(5000);
    bodyma(-87500,70000); //90 degree : 87000 (big number to avoid xtion sceen right side)
    //Check_MotorPE(1,2);


    XtionMotor(0,0); //motor 01
    bodyact("fwd");
    sleep(5);


}

void action_5f()
{
    // int ds2_1=0,ds2_2=0,ds2_3=0;

    // S5
    bodyJOGvel(1000,1000);
    sleep(1);

    printf("S5 \n");
//    int counter = 1550;
//
//    while(counter>1500)
//		{
//
//		    counter = 0;
//            streamDepth.readFrame( &frameDepth );
//            for(int i = 0 ; i<frameDepth.getHeight()/2;i++){
//                counter += depthMD(320,i);
//            }
//            counter = (int)(counter/(frameDepth.getHeight()/2));
//            printf("counter = %d\n", counter);
//
//			bodyact("fwd");
//			usleep(300000);
//
//			// printf("Wall distance: %d\n",d);
//		}
//   // while(counter>1500)
//   // {
//
//       // counter = 0;
//       // streamDepth.readFrame( &frameDepth );
//       // for(int i = 0 ; i<frameDepth.getWidth()/2;i++){
//           // counter += depthMD(i,300);
//       // }
//       // counter = (int)(counter/(frameDepth.getWidth()/2));
//       // printf("counter = %d\n", counter);
//
//           m9_1_d1=depthMD(300,240);
//           m9_1_d2=depthMD(320,240);
//           m9_1_d3=depthMD(330,240);
//           printf("d1=%d d2=%d  d3=%d \n",m9_1_d1,m9_1_d2,m9_1_d3);
//       // bodyact("fwd");
//       // usleep(300000);
//
//            //printf("Wall distance: %d\n",d);
//   // }
//   // while(1)
//   // {
//       // streamDepth.readFrame( &frameDepth );
//       // ds2_1=depthMD(320,230);
//       // ds2_2=depthMD(320,240);
//       // ds2_3=depthMD(320,250);
//
//       // if(ds2_1>1500 || ds2_2>1500 || ds2_3>1500 )
//       // {
//           // bodyact("fwd");
//           // usleep(100000);
//       // }
//       // else
//       // {
//           // bodyact("stop");
//           // break;
//       // }
//   // }


        int counter=0,counter_L=0,counter_R=0;
        int counter_abs=50;
        int while_juge=1550;
        int key=0,key1=0;

		bodyJOGvel(600,600); //700
		usleep(5000);
		bodyact("fwd");
        usleep(5000);

        //printf("w_j=%d \n",while_juge);
        while(while_juge>1500) //Reference Y-axis (Stairs 4f: 750, 5f:)
		{
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

			/*---Action---*/
            if(counter_abs<=3)
            {
                if(key1!=1 && counter>=1600)
                {
                    key1=1;
                    bodyJOGvel(900,900); //600
                    // bodyact("stop");
                    // usleep(5000);
                    // bodyact("fwd");
					usleep(5000);
                }
                if(counter<1600)
                {
                    bodyact("stop");
                    usleep(5000);
                    break;
                }
            }
            else if(counter_abs>=4 && counter_abs<=19)
            {
                if(key!=1 && counter_L<counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Right\n",counter_L,counter_R);
                    key=1;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(750,900);
                    //bodyact("right");
                    usleep(5000);
                }
                else if(key!=2 && counter_L>counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Left\n",counter_L,counter_R);
                    key=2;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(900,750);
                    //bodyact("left");
                    usleep(5000);
                }
            }
			else if(counter_abs>=20 && counter_abs<=30)
            {
                if(key!=3 && counter_L<counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Right\n",counter_L,counter_R);
                    key=3;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(650,900); //350,600
                    //bodyact("right");
                    usleep(5000);
                }
                else if(key!=4 && counter_L>counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Left\n",counter_L,counter_R);
                    key=4;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(900,650); //600,350
                    //bodyact("left");
                    usleep(5000);
                }
            }
			else if(counter_abs>=31)
            {
                if(key!=5 && counter_L<counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Right\n",counter_L,counter_R);
                    key=5;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(550,900); //250,600
                    //bodyact("right");
                    usleep(5000);
                }
                else if(key!=6 && counter_L>counter_R)
                {
                    //printf("counter_L= %d, counter_R= %d , Mode9-1 Need Left\n",counter_L,counter_R);
                    key=6;
                    key1=0;
                    //bodyact("stop");
                    //usleep(5000);//0.2
                    bodyJOGvel(900,550); //600.250
                    //bodyact("left");
                    usleep(5000);
                }
            }
		}


    // S6: go to 5F wall following ready
    printf("S6 \n");
    bodyJOGvel(500,500);
    bodyact("right");
    usleep(4900000); //9800
    bodyact("stop");
    usleep(200000);//right 90 degree

    bodyJOGvel(1000,1000);//go street
    sleep(1);
    bodyact("fwd");
    sleep(3); //sec
    bodyact("stop");
    sleep(1);

    // Wall Following 03
    bodyJOGvel(1000,1000);
    legMAvel(200,200);
    usleep(100000);

	XtionMotor(0,500); //motor 01
    usleep(3000000);
    WallFollowing_RightWall();

    bodyact("stop");
    sleep(1);
    XtionMotor(0,0);
    sleep(1);
    // end
    //sleep(20);

}




