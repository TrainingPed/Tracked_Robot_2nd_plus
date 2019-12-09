
char command[50]="";
char reader_response[50] = "";
/*-------Basic Function ----------*/
// Writter
void writter(char *Command,int fd_Slim5);
// Reader
void reader(int fd_Slim5);
/*-------------------------------*/

/*-------Motor Function ----------*/
// Motor Maxspeed
void Motor_speed(int motor_number, int motor_speed);
// Motor Velocity
void Motor_vel(int motor_number, int motor_vel);
// Motor Acc
void Motor_acc(int motor_number, int motor_acc);
// Motor Dec
void Motor_dec(int motor_number, int motor_dec);
// Motor Load Absolute Position
void Motor_LA(int motor_number, int motor_LA);
// Motor Load Relative Position
void Motor_LR(int motor_number, int motor_LR);
// Activate position control and start positioning
void Motor_M(int motor_number);
// Define Home Position
void Motor_HO(int motor_number, int motor_HO);
// Enable Motor
void Motor_EN(int motor_number);
// Disable Motor
void Motor_DI(int motor_number);
// Get Motor Actual Current Consumption
int Motor_GAC(int motor_number);

void writter2(char *Command, int fd_Slim5);
/*-------------------------------*/

void writter(char *Command, int fd_Slim5) 
{
	tcflush(fd_Slim5, TCIOFLUSH); //Flush Buffer data need some delay 
	usleep(2000);
	memset(Writtenbuf, '\0', sizeof(Writtenbuf));
	for (int i = 0; i < sizeof(Writtenbuf); i++)
		Writtenbuf[i] = Command[i];
	//std::cout <<"Command:" << command << "\n";
	int wordsWritten = write(fd_Slim5, Writtenbuf, sizeof(Writtenbuf));
	//reader(fd_Slim5);					// Read system message , preventing message left in encoder
//	std::cout << "fd = " << fd_Slim5 << "\n";
//	std::cout << "writter!!!\n";
	//below is reader
//	std::cout << "reader!!!start\n";
// 	int n = 0;
// 	int trytime = 0;

// 	char *pointer = reader_response;
// 	memset(reader_response, '\0', sizeof reader_response);//set response to null

// 	while ( trytime<50)   //read command to response 
// 	{
// 		n = read(fd_Slim5, pointer, 1);//reader_response + sizeof(reader_response) - pointer - 1
// 		if (n <= 0)
// 			break;

// 		trytime++;
// //		std::cout << "what is it ??" << reader_response << " pointer is " << *pointer << std::endl;
// 		if (trytime > 50)//timeout 
// 		{
// 			std::cout << "\n try time out break ...\n";
// 			break;
// 		}

// 		pointer+=n;
// 		if (pointer[-1] == '\n')
// 			break;
// 	}
// //	std::cout << "reader!!!end\n";

// 	if (n < 0)
// 	{
// 		std::cout << "Error reading: " << strerror(errno) << std::endl;
// 	}
// 	else if (n == 0)
// 	{
// 		std::cout << "Read nothing!" << std::endl;
// 	}
// 	else
// 	{
// 		std::cout << reader_response << std::endl; //Print the System message
// 	}
}
void writter2(char *Command, int fd_Slim5) 
{
	tcflush(fd_Slim5, TCIOFLUSH); //Flush Buffer data need some delay 
	usleep(2000);
	memset(Writtenbuf, '\0', sizeof(Writtenbuf));
	for (int i = 0; i < sizeof(Writtenbuf); i++)
		Writtenbuf[i] = Command[i];
	//std::cout <<"Command:" << command << "\n";
	int wordsWritten = write(fd_Slim5, Writtenbuf, sizeof(Writtenbuf));
	//reader(fd_Slim5);					// Read system message , preventing message left in encoder
//	std::cout << "fd = " << fd_Slim5 << "\n";
//	std::cout << "writter!!!\n";
	//below is reader
	std::cout << "reader!!!start\n";
	int n = 0;
	int trytime = 0;

	char *pointer = reader_response;
	memset(reader_response, '\0', sizeof reader_response);//set response to null

         //std::cout<<"pointer:"<<std::endl;
	 while(n=read(fd_Slim5,pointer,1)>0)
        {
		//std::cout<<"Reading... \n";
		pointer++;
		if (pointer[-1] == '\n')
			break;
		if (n<=0)
			break;
		std::cout<<*pointer;
			
	}

	std::cout << "reader!!!end\n";

	if (n < 0)
	{
		std::cout << "Error reading: " << strerror(errno) << std::endl;
	}
	else if (n == 0)
	{
		std::cout << "Read nothing!" << std::endl;
	}
	else
	{
		std::cout << reader_response << std::endl; //Print the System message
	}
}
void reader(int fd_Slim5 )
{
	std::cout << "reader!!!start\n";
	int n = 0;
	int trytime = 0;
	

	char *pointer =  reader_response;

	memset(reader_response, '\0', sizeof reader_response);//set response to null

	 while(n=read(fd_Slim5,pointer,1)>0)
    {
		//std::cout<<"Reading... \n";
		pointer++;
		if (pointer[-1] == '\n')
			break;
		if (n<=0)
			break;
			
	}
	if (n < 0)
	{
		std::cout << "Error reading: " << strerror(errno) << std::endl;
	}
	else if (n == 0)
	{
		std::cout << "Read nothing!" << std::endl;
	}
	else
	{
		std::cout << reader_response << std::endl; //Print the System message
	}


	
}

void Motor_speed(int motor_number, int motor_speed)
{
        memset(command, '\0', sizeof(command));				//Set command to null
	sprintf(command, "%dSP%d\r\n", motor_number, motor_speed);	//Editting command :Max speed
        writter2(command, fd_Slim5);					//Writting command
	//reader(fd_Slim5); 
}

void Motor_vel(int motor_number,int motor_speed)
{
	memset(command, '\0', sizeof(command));						//Set command to null
	sprintf(command, "%dV%d\r\n", motor_number, motor_speed);	//Editting command :Velocity
	writter2(command, fd_Slim5);									//Writting command
	//reader(fd_Slim5); 
}

void Motor_acc(int motor_number, int motor_acc)
{
	memset(command, '\0', sizeof(command));						//Set command to null
	sprintf(command, "%dAC%d\r\n", motor_number, motor_acc);	//Editting command :Acceleration
	writter2(command, fd_Slim5);									//Writting command
	//reader(fd_Slim5); 
}
void Motor_dec(int motor_number, int motor_dec)
{
	memset(command, '\0', sizeof(command));						//Set command to null
	sprintf(command, "%dDEC%d\r\n", motor_number, motor_dec);	//Editting command :Deceleration
	writter2(command, fd_Slim5);									//Writting command
	//reader(fd_Slim5); 
}

void Motor_LA(int motor_number, int motor_LA)
{
	memset(command, '\0', sizeof(command));						//Set command to null
	sprintf(command, "%dLA%d\r\n", motor_number, motor_LA);		//Editting command :Load Absolute Position
	writter2(command, fd_Slim5);									//Writting command
	//reader(fd_Slim5); 
}

void Motor_LR(int motor_number, int motor_LR)
{
	memset(command, '\0', sizeof(command));						//Set command to null
	sprintf(command, "%dLR%d\r\n", motor_number, motor_LR);		//Editting command :Load Relative Position
	writter2(command, fd_Slim5);									//Writting command
	//reader(fd_Slim5); 
}
void Motor_M(int motor_number)
{
	memset(command, '\0', sizeof(command));						//Set command to null
	sprintf(command, "%dM\r\n", motor_number);					//Editting command :Activate Position
	writter2(command, fd_Slim5);									//Writting command
	//reader(fd_Slim5); 
}
void Motor_HO(int motor_number, int motor_HO)
{
	memset(command, '\0', sizeof(command));						//Set command to null
	sprintf(command, "%dHO%d\r\n", motor_number, motor_HO);		//Editting command :Define Home Position
	writter2(command, fd_Slim5);									//Writting command
	//reader(fd_Slim5); 
}
void Motor_EN(int motor_number)
{
	memset(command, '\0', sizeof(command));						//Set command to null
	sprintf(command, "%dEN\r\n", motor_number);					//Editting command :Enable Motor
	writter2(command, fd_Slim5);								//Writting command
}
void Motor_DI(int motor_number)
{
	memset(command, '\0', sizeof(command));						//Set command to null
	sprintf(command, "%dDI\r\n", motor_number);					//Editting command :Disable Motor
        writter2(command, fd_Slim5);							//Writting command
}
int Motor_GAC(int motor_number)
{
    char result[100];
    memset(result, '\0', sizeof(result));
    memset(command, '\0', sizeof(command));					//Set command to null
    sprintf(command, "%dGRC\r\n", motor_number);				//Editting command :Get Actual Current
    writter(command, fd_Slim5);
    reader(fd_Slim5);
    int val = atoi(reader_response);
//    std::string str = reader_response;
//    std::cout<<"result = "<<str<<"\n";
    return val;


}



