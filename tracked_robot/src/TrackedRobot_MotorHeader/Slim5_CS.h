/*---Set Motor Coordinates---*/


void bodycs(int v1,int v2);	//Set T1,T2 Coordinates
void legcs(int v1,int v2);	//Set T0,T3 Coordinates


void bodycs(int v1,int v2)
{
    memset(Writtenbuf, '\0', sizeof(Writtenbuf));
    sprintf(Writtenbuf,"2HO%d\r\n",v1);
    int wordsWritten = write(fd_Slim5,Writtenbuf,sizeof(Writtenbuf));
    tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer data
    usleep(100000);

    memset(Writtenbuf, '\0', sizeof(Writtenbuf));
    sprintf(Writtenbuf,"3HO%d\r\n",v2);
    wordsWritten = write(fd_Slim5,Writtenbuf,sizeof(Writtenbuf));
    tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer data
}

void legcs(int v1,int v2)
{
    memset(Writtenbuf, '\0', sizeof(Writtenbuf));
    sprintf(Writtenbuf,"4HO%d\r\n",v1);
    int wordsWritten = write(fd_Slim5,Writtenbuf,sizeof(Writtenbuf));
    tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer data
    usleep(100000);

    memset(Writtenbuf, '\0', sizeof(Writtenbuf));
    sprintf(Writtenbuf,"1HO%d\r\n",v2);
    wordsWritten = write(fd_Slim5,Writtenbuf,sizeof(Writtenbuf));

    tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer data
}

