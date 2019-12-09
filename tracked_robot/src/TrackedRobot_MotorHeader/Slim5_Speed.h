/*---Set Motor Velocity---*/

//JOG Speed
void bodyJOGvel(int v1,int v2);	//Set T1,T2 JOG Speed (rang:0~4000)
void legJOGvel(int v1,int v2);	//Set T0,T3 JOG Speed (rang:0~4000)


void bodyJOGvel_select(int motor_number,int motor_speed);   //Choose T1 or T2 to control speed.
void bodyJOGvel_select(int motor_number,double motor_speed);

//MA Speed
void bodyMAvel(int v1,int v2);	//Set T1,T2 MA Speed (rang:0~4000) //2,3
void legMAvel(int v1,int v2);	//Set T0,T3 MA Speed (rang:0~4000) //4,1

//============================

//Robot Acceleration Set
void bodyva(int v1, int v2);	//Set T1,T2 Acceleration	(rang:0~1000)
void legva(int v1, int v2);		//Set T0,T3 Acceleration	(rang:0~1000)
/*---------JG.h--------------*/

bool bodyact(char *ac1);	//T1,T2 Move //2,3
bool legact(char *ac1,char *ac2);	//T0,T3 Move  //4,1

bool Left_Tracked(double motor_speed);
bool Right_Tracked(double motor_speed);

//=============================

/*----------MA.h-------------*/
//int Globle_T1MA_Position = 0, Globle_T2MA_Position = 0;
//int Globle_T0MA_Position = 0, Globle_T3MA_Position = 0;

void bodyma(int v1, int v2);
void legma(int v1, int v2);

/*---------------------------*/

/*----------MR.h-------------*/
void bodymr(int v1, int v2);
/*---------------------------*/

/*-------speed limit-------------*/
int bodyvel_left,bodyvel_right;
int legvel_front, legvel_back;
int bodyMAvel_left, bodyMAvel_right;
int legMAvel_front, legMAvel_back;
/*-------------------------------*/

void bodyJOGvel_select(int motor_number,int motor_speed)
{
    memset(Writtenbuf, '\0', sizeof(Writtenbuf));
    sprintf(Writtenbuf,"%dSP%d\r\n",motor_number,motor_speed);
    int wordsWritten = write(fd_Slim5,Writtenbuf,sizeof(Writtenbuf));
    tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer data
    usleep(2000);
    memset(Writtenbuf, '\0', sizeof(Writtenbuf));
    sprintf(Writtenbuf,"%dV%d\r\n",motor_number,motor_speed);
    wordsWritten = write(fd_Slim5,Writtenbuf,sizeof(Writtenbuf));
    tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer data
    usleep(2000);

}

void bodyJOGvel_select(int motor_number,double motor_speed)
{
     if(motor_speed==0)
    {
        return ;
    }

    motor_speed = abs(motor_speed);

    if( ((int)(motor_speed * 10) % 10) >= 5)
        motor_speed = abs((int)(motor_speed + 1));
    else
        motor_speed = (int)(motor_speed);


    memset(Writtenbuf, '\0', sizeof(Writtenbuf));
    sprintf(Writtenbuf,"%dSP%d\r\n",motor_number,(int)(motor_speed));
    int wordsWritten = write(fd_Slim5,Writtenbuf,sizeof(Writtenbuf));
    tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer data
    usleep(2000);sprintf(Writtenbuf,"%dV%d\r\n",motor_number,(int)motor_speed);
    wordsWritten = write(fd_Slim5,Writtenbuf,sizeof(Writtenbuf));
    tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer data
    usleep(2000);



}

void bodyJOGvel(int v1,int v2)
{
	bodyvel_left = v1;
	bodyvel_right = v2;
    memset(Writtenbuf, '\0', sizeof(Writtenbuf));
    sprintf(Writtenbuf,"2SP%d\r\n",v1);
    int wordsWritten = write(fd_Slim5,Writtenbuf,sizeof(Writtenbuf));
    tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer data
    usleep(2000);

    memset(Writtenbuf, '\0', sizeof(Writtenbuf));
    sprintf(Writtenbuf,"3SP%d\r\n",v2);
    wordsWritten = write(fd_Slim5,Writtenbuf,sizeof(Writtenbuf));
    tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer data
}



void legJOGvel(int v1,int v2)
{
	legvel_front = v2;
	legvel_back = v1;
    memset(Writtenbuf, '\0', sizeof(Writtenbuf));
    sprintf(Writtenbuf,"4SP%d\r\n",v1);
    int wordsWritten = write(fd_Slim5,Writtenbuf,sizeof(Writtenbuf));
    tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer data
    usleep(2000);

    memset(Writtenbuf, '\0', sizeof(Writtenbuf));
    sprintf(Writtenbuf,"1SP%d\r\n",v2);
    wordsWritten = write(fd_Slim5,Writtenbuf,sizeof(Writtenbuf));

    tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer data
}

void bodyMAvel(int v1,int v2)
{
	bodyMAvel_left = v1;
	bodyMAvel_right = v2;
    memset(Writtenbuf, '\0', sizeof(Writtenbuf));
    sprintf(Writtenbuf,"2SP%d\r\n",v1);
    int wordsWritten = write(fd_Slim5,Writtenbuf,sizeof(Writtenbuf));
    tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer data
    usleep(2000);

    memset(Writtenbuf, '\0', sizeof(Writtenbuf));
    sprintf(Writtenbuf,"3SP%d\r\n",v2);
    wordsWritten = write(fd_Slim5,Writtenbuf,sizeof(Writtenbuf));

    tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer data
}

void legMAvel(int v1,int v2)
{
	legMAvel_back = v1;
	legMAvel_front = v2;
    memset(Writtenbuf, '\0', sizeof(Writtenbuf));
    sprintf(Writtenbuf,"4SP%d\r\n",v1);
    int wordsWritten = write(fd_Slim5,Writtenbuf,sizeof(Writtenbuf));
    tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer data
    usleep(2000);

    memset(Writtenbuf, '\0', sizeof(Writtenbuf));
    sprintf(Writtenbuf,"1SP%d\r\n",v2);
    wordsWritten = write(fd_Slim5,Writtenbuf,sizeof(Writtenbuf));

    tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer data
}

void bodyva(int v1,int v2)
{
	memset(Writtenbuf, '\0', sizeof(Writtenbuf));
    sprintf(Writtenbuf,"2AC%d\r\n",v1);
    int wordsWritten = write(fd_Slim5,Writtenbuf,sizeof(Writtenbuf));
    tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer data
    usleep(2000);
	memset(Writtenbuf, '\0', sizeof(Writtenbuf));
	sprintf(Writtenbuf, "2DEC%d\r\n", v1);
	wordsWritten = write(fd_Slim5, Writtenbuf, sizeof(Writtenbuf));
	tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer data
	usleep(2000);
	memset(Writtenbuf, '\0', sizeof(Writtenbuf));
	sprintf(Writtenbuf, "3AC%d\r\n", v2);
	wordsWritten = write(fd_Slim5, Writtenbuf, sizeof(Writtenbuf));
	tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer data
	usleep(2000);
	memset(Writtenbuf, '\0', sizeof(Writtenbuf));
	sprintf(Writtenbuf, "3DEC%d\r\n", v2);
	wordsWritten = write(fd_Slim5, Writtenbuf, sizeof(Writtenbuf));
	tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer data
	usleep(2000);


}

void legva(int v1, int v2)
{

	memset(Writtenbuf, '\0', sizeof(Writtenbuf));
	sprintf(Writtenbuf, "4AC%d\r\n", v1);
	int wordsWritten = write(fd_Slim5, Writtenbuf, sizeof(Writtenbuf));
	tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer data
	usleep(2000);
	memset(Writtenbuf, '\0', sizeof(Writtenbuf));
	sprintf(Writtenbuf, "4DEC%d\r\n", v1);
	wordsWritten = write(fd_Slim5, Writtenbuf, sizeof(Writtenbuf));
	tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer data
	usleep(2000);
	memset(Writtenbuf, '\0', sizeof(Writtenbuf));
	sprintf(Writtenbuf, "1DEC%d\r\n", v2);
	wordsWritten = write(fd_Slim5, Writtenbuf, sizeof(Writtenbuf));
	tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer data
	usleep(2000);
	memset(Writtenbuf, '\0', sizeof(Writtenbuf));
	sprintf(Writtenbuf, "1DEC%d\r\n", v2);
	wordsWritten = write(fd_Slim5, Writtenbuf, sizeof(Writtenbuf));
	tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer data
	usleep(2000);
}
/*-----------------------JG.h-------------------------------------*/
bool bodyact(char *ac1)
{
    int wordsWritten=-1;

RE_bodyact:

    memset(Writtenbuf, '\0', sizeof(Writtenbuf));

    if(ac1=="fwd")
    {

		sprintf(Writtenbuf, "\r\n2V-%d\r\n", bodyvel_left);
        wordsWritten = write(fd_Slim5,Writtenbuf,sizeof(Writtenbuf));
        tcflush(fd_Slim5, TCIFLUSH);
        usleep(2000);

		sprintf(Writtenbuf, "\r\n3V%d\r\n", bodyvel_right);
        wordsWritten = write(fd_Slim5,Writtenbuf,sizeof(Writtenbuf));
		tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer data
        usleep(2000);

    }
//
    else if(ac1=="back")
    {
		sprintf(Writtenbuf, "\r\n2V%d\r\n", bodyvel_left);
		wordsWritten = write(fd_Slim5, Writtenbuf, sizeof(Writtenbuf));
		tcflush(fd_Slim5, TCIFLUSH);
		usleep(1000);

		sprintf(Writtenbuf, "\r\n3V-%d\r\n", bodyvel_right);
		wordsWritten = write(fd_Slim5, Writtenbuf, sizeof(Writtenbuf));
		tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer data
		usleep(1000);
    }

    else if(ac1=="left")
    {
		sprintf(Writtenbuf, "\r\n2V%d\r\n", bodyvel_left);
		wordsWritten = write(fd_Slim5, Writtenbuf, sizeof(Writtenbuf));
		tcflush(fd_Slim5, TCIFLUSH);
		usleep(1000);

		sprintf(Writtenbuf, "\r\n3V%d\r\n", bodyvel_right);
		wordsWritten = write(fd_Slim5, Writtenbuf, sizeof(Writtenbuf));
		tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer data
		usleep(1000);
    }

    else if(ac1=="right")
    {
		sprintf(Writtenbuf, "\r\n2V-%d\r\n", bodyvel_left);
		wordsWritten = write(fd_Slim5, Writtenbuf, sizeof(Writtenbuf));
		tcflush(fd_Slim5, TCIFLUSH);
		usleep(1000);

		sprintf(Writtenbuf, "\r\n3V-%d\r\n", bodyvel_right);
		wordsWritten = write(fd_Slim5, Writtenbuf, sizeof(Writtenbuf));
		tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer data
		usleep(1000);
    }

    else if(ac1=="stop")
    {
        sprintf(Writtenbuf,"\r\nV0\r\n");
        wordsWritten = write(fd_Slim5,Writtenbuf,sizeof(Writtenbuf));
        usleep(1000);
        tcflush(fd_Slim5, TCIFLUSH);
        usleep(1000);


//        write(fd_Slim5,"STOP\r\n",6);
//        write(fd_Slim5,"T1STOP\r\n",8);
//        write(fd_Slim5,"T2STOP\r\n",8);
//        write(fd_Slim5,"T3STOP\r\n",8);
    }
    else
        return false;


    tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer data
    usleep(5000);
    //usleep(200000);


    if(wordsWritten <= 0)
        goto RE_bodyact;

    return true;

}
bool legact(char *ac1,char *ac2)
{
    int wordsWritten = -1;

RE_legact:

    memset(Writtenbuf, '\0', sizeof(Writtenbuf));

    if(ac1=="fwd")
    {
		sprintf(Writtenbuf, "\r\n4V-%d\r\n", legMAvel_back);
        wordsWritten = write(fd_Slim5,Writtenbuf,sizeof(Writtenbuf));
    }
    else if(ac1=="rev")
    {
		sprintf(Writtenbuf, "\r\n4V%d\r\n", legMAvel_back);
        wordsWritten = write(fd_Slim5,Writtenbuf,sizeof(Writtenbuf));
    }
    else if(ac2=="fwd")
    {
		sprintf(Writtenbuf, "\r\n1V-%d\r\n", legMAvel_front);
        wordsWritten = write(fd_Slim5,Writtenbuf,sizeof(Writtenbuf));
    }
    else if(ac2=="rev")
    {
		sprintf(Writtenbuf, "\r\n1V%d\r\n", legMAvel_front);
        wordsWritten = write(fd_Slim5,Writtenbuf,sizeof(Writtenbuf));
    }
    else if(ac1=="stop" && ac2=="stop")
    {
        sprintf(Writtenbuf,"\r\nV0\r\n");
        wordsWritten = write(fd_Slim5,Writtenbuf,sizeof(Writtenbuf));
    }
    else
        return false;

    tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer data
    //usleep(1000);
    usleep(2000);

    if(wordsWritten <= 0)
        goto RE_legact;

    return true;
}


bool Left_Tracked(double motor_speed)
{
    int wordsWritten=-1;

//RE_Left_Tracked1:
//
//
//    if(Speed != 0)
//    {
//        memset(Writtenbuf, '\0', sizeof(Writtenbuf));
//        sprintf(Writtenbuf,"T1PN14=%d\r\n",V_speed);
//        wordsWritten = write(fd_Slim5,Writtenbuf,sizeof(Writtenbuf));
//        usleep(1000);
//        tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer data
//        usleep(1000);
//        if(wordsWritten <= 0)
//            goto RE_Left_Tracked1;
//    }


RE_Left_Tracked2:

    memset(Writtenbuf, '\0', sizeof(Writtenbuf));

   if(motor_speed > 0)
   {
        sprintf(Writtenbuf,"\r\n2V-%d\r\n",bodyvel_left);
        wordsWritten = write(fd_Slim5,Writtenbuf,sizeof(Writtenbuf));
        usleep(1000);
        tcflush(fd_Slim5, TCIFLUSH);
        usleep(1000);
   }
   else if(motor_speed < 0)
   {
        sprintf(Writtenbuf,"\r\n2V%d\r\n",bodyvel_left);
        wordsWritten = write(fd_Slim5,Writtenbuf,sizeof(Writtenbuf));
        usleep(1000);
        tcflush(fd_Slim5, TCIFLUSH);
        usleep(1000);
   }
   else if(motor_speed == 0)
   {
        sprintf(Writtenbuf,"\r\n2V0\r\n");
        wordsWritten = write(fd_Slim5,Writtenbuf,sizeof(Writtenbuf));
        usleep(1000);
        tcflush(fd_Slim5, TCIFLUSH);
        usleep(1000);
   }

   else
        return false;


    if(wordsWritten <= 0)
        goto RE_Left_Tracked2;

    return true;
}

bool Right_Tracked(double motor_speed)
{
    int wordsWritten=-1;

//RE_Right_Tracked1:
//
//
//    if(Speed != 0)
//    {
//        memset(Writtenbuf, '\0', sizeof(Writtenbuf));
//        sprintf(Writtenbuf,"T2PN14=%d\r\n",V_speed);
//        wordsWritten = write(fd_Slim5,Writtenbuf,sizeof(Writtenbuf));
//        usleep(1000);
//        tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer data
//        usleep(1000);
//        if(wordsWritten <= 0)
//            goto RE_Right_Tracked1;
//    }


RE_Right_Tracked2:

    memset(Writtenbuf, '\0', sizeof(Writtenbuf));

   if(motor_speed > 0)
   {
        sprintf(Writtenbuf,"\r\n3V%d\r\n",bodyvel_right);
        wordsWritten = write(fd_Slim5,Writtenbuf,sizeof(Writtenbuf));
        usleep(1000);
        tcflush(fd_Slim5, TCIFLUSH);
        usleep(1000);
   }
   else if(motor_speed < 0)
   {
        sprintf(Writtenbuf,"\r\n3V-%d\r\n",bodyvel_right);
        wordsWritten = write(fd_Slim5,Writtenbuf,sizeof(Writtenbuf));
        usleep(1000);
        tcflush(fd_Slim5, TCIFLUSH);
        usleep(1000);
   }
   else if(motor_speed == 0)
   {
        sprintf(Writtenbuf,"\r\n3V0\r\n");
        wordsWritten = write(fd_Slim5,Writtenbuf,sizeof(Writtenbuf));
        usleep(1000);
        tcflush(fd_Slim5, TCIFLUSH);
        usleep(1000);
   }

   else
        return false;


    if(wordsWritten <= 0)
        goto RE_Right_Tracked2;

    return true;
}
/*-----------------------------------------------------------*/
/*---------------------MA.h----------------------------------*/

void bodyma(int v1, int v2)
{
	//Globle_T1MA_Position = v1;
	//Globle_T2MA_Position = v2;

	memset(Writtenbuf, '\0', sizeof(Writtenbuf));
	sprintf(Writtenbuf, "2SP%d\r\n", bodyMAvel_left);
	int wordsWritten = write(fd_Slim5, Writtenbuf, sizeof(Writtenbuf));
	tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer data
	usleep(50000);

	memset(Writtenbuf, '\0', sizeof(Writtenbuf));
	sprintf(Writtenbuf, "3SP%d\r\n", bodyMAvel_right);
	wordsWritten = write(fd_Slim5, Writtenbuf, sizeof(Writtenbuf));
	tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer data
	usleep(5000);

	memset(Writtenbuf, '\0', sizeof(Writtenbuf));
	sprintf(Writtenbuf, "\r\n2LA-%d\r\n", v1);
	wordsWritten = write(fd_Slim5, Writtenbuf, sizeof(Writtenbuf));
	//printf("wordsWritten = %d\n",wordsWritten);
	tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer data
	usleep(5000);

	memset(Writtenbuf, '\0', sizeof(Writtenbuf));
	sprintf(Writtenbuf, "\r\n3LA%d\r\n", v2);
	wordsWritten = write(fd_Slim5, Writtenbuf, sizeof(Writtenbuf));
	tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer data
	usleep(5000);


	memset(Writtenbuf, '\0', sizeof(Writtenbuf));
	sprintf(Writtenbuf, "\r\n2M\r\n");
	wordsWritten = write(fd_Slim5, Writtenbuf, sizeof(Writtenbuf));
	//printf("wordsWritten = %d\n",wordsWritten);
    tcflush(fd_Slim5, TCIFLUSH);

	usleep(5000);
	memset(Writtenbuf, '\0', sizeof(Writtenbuf));
	sprintf(Writtenbuf, "\r\n3M\r\n");
	wordsWritten = write(fd_Slim5, Writtenbuf, sizeof(Writtenbuf));
	//printf("wordsWritten = %d\n",wordsWritten);
    tcflush(fd_Slim5, TCIFLUSH);

	usleep(5000);


}

void legma(int v1,int v2)
{
	v1=v1*-1;
	v2=v2*-1;
	memset(Writtenbuf, '\0', sizeof(Writtenbuf));
	sprintf(Writtenbuf, "\r\n1SP%d\r\n", legMAvel_front);
	int wordsWritten = write(fd_Slim5, Writtenbuf, sizeof(Writtenbuf));
	tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer data
	usleep(5000);

	memset(Writtenbuf, '\0', sizeof(Writtenbuf));
	sprintf(Writtenbuf, "\r\n4SP%d\r\n", legMAvel_back);
	wordsWritten = write(fd_Slim5, Writtenbuf, sizeof(Writtenbuf));
	tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer data
	usleep(5000);

	memset(Writtenbuf, '\0', sizeof(Writtenbuf));
	sprintf(Writtenbuf, "\r\n1LA%d\r\n", v2);
	wordsWritten = write(fd_Slim5, Writtenbuf, sizeof(Writtenbuf));
	//printf("wordsWritten = %d\n",wordsWritten);
	tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer dat
	usleep(5000);
	memset(Writtenbuf, '\0', sizeof(Writtenbuf));
	sprintf(Writtenbuf, "\r\n4LA%d\r\n", v1);
	wordsWritten = write(fd_Slim5, Writtenbuf, sizeof(Writtenbuf));
	//printf("wordsWritten = %d\n",wordsWritten);

	tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer dat
	usleep(5000);


	memset(Writtenbuf, '\0', sizeof(Writtenbuf));
	sprintf(Writtenbuf, "\r\n1M\r\n");
	wordsWritten = write(fd_Slim5, Writtenbuf, sizeof(Writtenbuf));
	//printf("wordsWritten = %d\n",wordsWritten);
	tcflush(fd_Slim5, TCIFLUSH);
	usleep(5000);
	memset(Writtenbuf, '\0', sizeof(Writtenbuf));
	sprintf(Writtenbuf, "\r\n4M\r\n");
	wordsWritten = write(fd_Slim5, Writtenbuf, sizeof(Writtenbuf));
	//printf("wordsWritten = %d\n",wordsWritten);
	tcflush(fd_Slim5, TCIFLUSH);
	usleep(5000);
}


/*-----------------------------------------------------------*/


/*---------------------MR.h----------------------------------*/
void bodymr(int v1, int v2)
{
	memset(Writtenbuf, '\0', sizeof(Writtenbuf));
	sprintf(Writtenbuf, "\r\n2LR-%d\r\n", v1);
	int wordsWritten = write(fd_Slim5, Writtenbuf, sizeof(Writtenbuf));
	tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer data
	usleep(5000);

	memset(Writtenbuf, '\0', sizeof(Writtenbuf));
	sprintf(Writtenbuf,"\r\n3LR%d\r\n",v2);
	wordsWritten = write(fd_Slim5,Writtenbuf,sizeof(Writtenbuf));
	tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer data
	usleep(5000);

	memset(Writtenbuf, '\0', sizeof(Writtenbuf));
	sprintf(Writtenbuf, "M\r\n");
	wordsWritten = write(fd_Slim5, Writtenbuf, sizeof(Writtenbuf));
	tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer data
	usleep(2000);


}

/*-----------------------------------------------------------*/
