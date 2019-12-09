/*---RS232 Include---*/
#include <string.h>
#include <unistd.h>     //Used for UART
#include <fcntl.h>      //Used for UART
#include <termios.h>    //Used for UART


/*---define---*/
#define serialPortName "/dev/Faulhaber"//"/dev/Faulhaber"
#define Xtion_serialPortName "/dev/MX28"

int fd_Slim5;

char Writtenbuf[50] = "";

/*---port set---*/
int Motor_initial(int fd_Slim5);
void Motor_close();

/*---motor reset---*/
void reset();

/*motor control*/
int Motor_initial(int fd_Slim5)
{
	//Use from USB-serial
    //Open in non blocking read/write mode
    
    //fd_Slim9 = open(Xtion_serialPortName,O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
	
	//  ----EVENT-----
	fcntl(fd_Slim5, F_SETFL, 0);//fd = File Descriptor 
	//  ----EVENT-----
	//This call will set opened file desciptor's file status flag to value 0.
	//The FNDELAY option causes the read function to return 0 if no characters are available on the port. To restore normal (blocking) behavior, call fcntl() without the FNDELAY option:

    //ERROR - CAN'T OPEN SERIAL PORT
    if(fd_Slim5<0)
    {
        printf("*** Open fd_Slim5 port error ***\n");
        return -1;
    }
    /*else if(fd_Slim9<0)
    {
        printf("*** Open fd_Slim9 port error ***\n");
        return -1;
    }*/

    //CONFIGURE THE UART
	//The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
	//	Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
	//	CSIZE:- CS5, CS6, CS7, CS8
	//	CLOCAL - Ignore modem status lines
	//	CREAD - Enable receiver
	//	IGNPAR = Ignore characters with parity errors
	//	ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
	//	PARENB - Parity enable
	//	PARODD - Odd parity (else even)


    struct termios options;
    tcgetattr(fd_Slim5, &options);
    //tcgetattr(fd_Slim9, &options);

    cfsetispeed(&options,B9600);
    cfsetospeed(&options,B9600);
//
//    options.c_ispeed = B9600;
//    options.c_ospeed = B9600;
//
//    options.c_cflag &= ~PARENB;
//    options.c_cflag &= ~CSTOPB;
//    options.c_cflag &= ~CSIZE;
//    options.c_cflag |= CS8;
//    options.c_cflag |= (CREAD);
//    options.c_iflag |= (IXON | IXOFF);

    //options.c_cflag = B9600 | CS8 | CLOCAL | CREAD ;
    //options.c_iflag = IGNPAR;
    //options.c_oflag = 0;
    //options.c_lflag = ICANON;
	//------------------------------------------------------
	/* Setting other Port Stuff */
	options.c_cflag &= ~PARENB;            // Make 8n1
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;

	options.c_cflag &= ~CRTSCTS;           // no flow control
        options.c_cc[VMIN] = 0;                // read doesn't block
        options.c_cc[VTIME] = 1;               // 0.5 seconds read timeout
	options.c_cflag |= CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

										   /* Make raw */
	cfmakeraw(&options);
	//-------------------------------------------------------

    if(tcsetattr(fd_Slim5, TCSANOW, &options)!=0)
    {
        printf("error %d from tcsetattr", errno);
        close(fd_Slim5);
        //close(fd_Slim9);
        return -1;
    }

    /*if(tcsetattr(fd_Slim9, TCSANOW, &options)!=0)
    {
        printf("error %d from tcsetattr", errno);
        close(fd_Slim9);
        close(fd_Slim5);
        return -1;
    }*/


	/* Flush Port, then applies attributes */
    tcflush(fd_Slim5, TCIOFLUSH); //Flush Buffer data
    //tcflush(fd_Slim9, TCIOFLUSH); //Flush Buffer data

    return 1;
}

void Motor_close()
{
    close(fd_Slim5);
	//close(fd_Slim9);
}


void reset()
{
    memset(Writtenbuf, '\0', sizeof(Writtenbuf));
    sprintf(Writtenbuf,"RESET\r\n");
    int wordsWritten = write(fd_Slim5,Writtenbuf,sizeof(Writtenbuf));

    tcflush(fd_Slim5, TCIFLUSH); //Flush Buffer data
}






