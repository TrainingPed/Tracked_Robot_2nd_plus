#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <errno.h>
#include <cstdio>
#include <iostream> //lib use in c++
#include <time.h>
#include <string.h>
#include <unistd.h>     //Used for UART
#include <fcntl.h>      //Used for UART
#include <termios.h>    //Used for UART
#include "std_msgs/Int32.h"

#define sensorName "/dev/ttymxc3"
#define baudrate B9600
int fd_Sensor;
//int rc;
int flags;
int Sensor_serialport_read(int fd, int num, char endsign)
{
    printf("!!\n");
    tcflush(fd, TCIOFLUSH);
	
    
	char buf01[num];    
	memset(buf01,'\0',sizeof buf01);
	
	char *pointer=buf01;
	
    int i=0;
	int n=0;
	
	write(fd,"0",1);
	//printf("write ok \n");
	//n=read(fd,buf01,num);
	////printf("read ok \n");
	//buf01[n]=0;
	usleep(1000);
    while(n=read(fd,pointer,1)>0)
    {
		//std::cout<<"Reading... \n";
		pointer++;
		if( pointer[-1] == endsign)
			break;
		if (n<=0)
			break;
			
	}
	if (n<0)
		std::cout<<"Error reading \n";
	else if(n==0)
		std::cout<<"Read Nothing!\n";
	

    
    int data=atoi(buf01); //atoi : convert char to int
    std::cout<<"ultrasonic data :"<<data<<"\n";
    
    return data;
}
int sensorData(int sensorNumber)
{
    //fd=open(sensorName,O_RDWR | O_NOCTTY | O_NDELAY);
    
    fd_Sensor=open(sensorName,O_RDWR | O_NOCTTY );
    
 
    //usleep(3500000);
    if(fd_Sensor<0)
    {
        printf("*** Open ttymxc3 port error ***\n");
        return -1;
    }
    

    struct termios options;
	memset (&options, 0, sizeof options);
        if (tcgetattr (fd_Sensor, &options) != 0)
        {
                printf ("error from tcgetattr", errno);
                return -1;
        }

    tcgetattr(fd_Sensor, &options);

	cfsetospeed (&options, baudrate);
    cfsetispeed (&options, baudrate);

	options.c_cflag = (options.c_cflag & ~CSIZE) | CS8;     // 8-bit chars

	options.c_iflag &= ~IGNBRK;         // ignore break signal
	options.c_lflag = 0;                // no signaling chars, no echo,
									    // no canonical processing
	options.c_oflag = 0;                // no remapping, no delays
	options.c_cc[VMIN]  = 0;            // read doesn't block
	options.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	options.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	options.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
									// enable reading
	options.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CRTSCTS;


    if( tcsetattr(fd_Sensor, TCSANOW, &options) <0)
    {
        printf("*** Error from sensor tcsetattr ***\n");
        close(fd_Sensor);
        return -1;
    }


    int sensorRead01=0;

  
		sensorRead01=Sensor_serialport_read(fd_Sensor,50,';');
		   

	  
		if(sensorNumber==1)
		{
		   printf("ultrasonic has read successfully \n");
		   
		   return sensorRead01;
		}
	

    tcflush(fd_Sensor, TCIOFLUSH); //Flush Buffer data

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Ultrasonic");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<std_msgs::Int32>("ultrasonic", 1000);
	ros::Rate loop_rate(10);
	sensorData(1);
	
	while(ros::ok())
	{
		std_msgs::Int32 sonic;
		flags=fcntl(fd_Sensor,F_GETFL,0);

			sonic.data=Sensor_serialport_read(fd_Sensor,50,';');
			//printf("Pub~~\n");
			pub.publish(sonic);
		
		loop_rate.sleep();
	}

	return 0;
}
