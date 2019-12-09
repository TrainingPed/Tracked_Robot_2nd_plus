#include <stdlib.h>
#include <string.h>

char response[50]="";

int n = 0;
int spot = 0;
int val = 0;


int Encoder(int enc , int fd_Slim5)
{
	int trytime = 0;
	//Writing command to the controller
	memset(Writtenbuf, '\0', sizeof(Writtenbuf));
	switch (enc)
	{
	case 1:
		sprintf(Writtenbuf, "1POS\r\n");
		break;
	case 2:
		sprintf(Writtenbuf, "2POS\r\n");
		break;
	case 3:
		sprintf(Writtenbuf, "3POS\r\n");
		break;
	case 4:
		sprintf(Writtenbuf, "4POS\r\n");
		break;
	default:
		break;
	}

	tcflush(fd_Slim5, TCIOFLUSH); //Flush Buffer data
	usleep(2000);
	int wordsWritten = write(fd_Slim5, Writtenbuf, sizeof(Writtenbuf));



	//below is reading ...
	
	//char temp[255];
	//char *tempPtr = temp;

	char *pointer = response;
	//char **doublePtr = &pointer;
	memset(response, '\0', sizeof response);//set response to null

//	std::cout << "Encoder start listen!\n";
	while (trytime<50)   //read command to response 
	{
		n = read(fd_Slim5, pointer, response + sizeof(response) - pointer - 1);
		if (n <= 0)
			break;

		trytime++;
//		std::cout << "what is it ??" << response << " pointer is " << *pointer << std::endl;
		if (trytime > 50 )//timeout 
		{
			std::cout << "\n try times out break ...\n";
			break;
		}
	
		pointer +=n;
		if (((pointer[-1] < '0') || (pointer[-1] > '9')) && (pointer[-1] != '\r') && (pointer[-1] != '\n') && (pointer[-1] != '-'))//nonlegal then break
		{
			std::cout << pointer[-1] << "\n";
			break;
		}
	//	assert(((pointer[-1] < '0') || (pointer[-1] > '9')) && (pointer[-1] != '\r') && (pointer[-1] != '\n') && (pointer[-1] != '-'));
		if (pointer[-1] == '\n' )
			break;

	}  
//	std::cout << "End listen!\n";


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
		
		val=atoi(response); //convert char array to integer
		//std::cerr  << response<<" <-response "<<val << std::endl;//<< "Response: "
	}
	


	return val;
}
