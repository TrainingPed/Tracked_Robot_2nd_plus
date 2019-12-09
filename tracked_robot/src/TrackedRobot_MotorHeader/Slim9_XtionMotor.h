/*---Slim9 Motor Control (Xtion Motor)---*/

int Globle_T0MA_Position = 0, Globle_T1MA_Position = 0;
void XtionMotor(int v2,int v1);
void XtionMotor_MR(int v2,int v1);
void XtionMotor_CS(int v2, int v1);
void XtionMotor_stop();

void XtionMotor(int v2,int v1)
{
    memset(Writtenbuf, '\0', sizeof(Writtenbuf));
    sprintf(Writtenbuf,"T0MA%d\r\n",v1);
	int wordsWritten = write(fd_Slim9,Writtenbuf,sizeof(Writtenbuf));
    tcflush(fd_Slim9, TCIFLUSH);
    usleep(1000);

	Globle_T0MA_Position = v1;

    //memset(Writtenbuf, '\0', sizeof(Writtenbuf));
    sprintf(Writtenbuf,"T1MA%d\r\n",v2);
	wordsWritten = write(fd_Slim9,Writtenbuf,sizeof(Writtenbuf));
    tcflush(fd_Slim9, TCIFLUSH); //Flush Buffer data
	usleep(1000);

	Globle_T1MA_Position = v2;
}
void XtionMotor_MR(int v2,int v1)
{
    memset(Writtenbuf, '\0', sizeof(Writtenbuf));
    sprintf(Writtenbuf,"T0MR%d\r\n",v1);
    int wordsWritten = write(fd_Slim9,Writtenbuf,sizeof(Writtenbuf));
    tcflush(fd_Slim9, TCIFLUSH);
    usleep(1000);


    //memset(Writtenbuf, '\0', sizeof(Writtenbuf));
    sprintf(Writtenbuf,"T1MR%d\r\n",v2);
    wordsWritten = write(fd_Slim9,Writtenbuf,sizeof(Writtenbuf));
    tcflush(fd_Slim9, TCIFLUSH); //Flush Buffer data
	usleep(1000);


}


void XtionMotor_stop()
{
    memset(Writtenbuf, '\0', sizeof(Writtenbuf));
    sprintf(Writtenbuf,"stop\r\n");
    int wordsWritten = write(fd_Slim9,Writtenbuf,sizeof(Writtenbuf));
    tcflush(fd_Slim9, TCIFLUSH); //Flu
    usleep(50000);
}
void XtionMotor_CS(int v2, int v1)
{
	memset(Writtenbuf, '\0', sizeof(Writtenbuf));
	sprintf(Writtenbuf, "T0CS%d\r\n", v2);
	int wordsWritten = write(fd_Slim9, Writtenbuf, sizeof(Writtenbuf));
	tcflush(fd_Slim9, TCIFLUSH);
	usleep(10000);
    Globle_T0MA_Position=v2;
	//memset(Writtenbuf, '\0', sizeof(Writtenbuf));
	sprintf(Writtenbuf, "T1CS%d\r\n", v2);
	wordsWritten = write(fd_Slim9, Writtenbuf, sizeof(Writtenbuf));
	tcflush(fd_Slim9, TCIFLUSH); //Flush Buffer data
	usleep(10000);
	Globle_T1MA_Position=v1;

}
