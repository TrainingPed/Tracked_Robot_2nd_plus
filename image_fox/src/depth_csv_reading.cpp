#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "stdio.h"
#include <vector>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>


/*
 * A class to read data from a csv file.
 */
class CSVReader
{
	std::string fileName;
	std::string delimeter;

public:
	CSVReader(std::string filename, std::string delm = ",") :
			fileName(filename), delimeter(delm)
	{ }

	// Function to fetch data from a CSV File
	std::vector<std::vector<std::string> > getData();
};

/*
* Parses through csv file line by line and returns the data
* in vector of vector of strings.
*/
std::vector<std::vector<std::string> > CSVReader::getData()
{
  
	std::ifstream file(fileName.c_str());

	std::vector<std::vector<std::string> > dataList;

	std::string line = "";
	// Iterate through each line and split the content using delimeter
	while (getline(file, line))
	{
		std::vector<std::string> vec;
		boost::algorithm::split(vec, line, boost::is_any_of(delimeter));
		dataList.push_back(vec);
	}
	// Close the File
	file.close();

	return dataList;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_csv_reading");

  ros::NodeHandle nh;

	// Creating an object of CSVWriter
    CSVReader reader("/home/cirlab/Desktop/depth_data1.csv");

	// Get the data from CSV File
	std::vector<std::vector<std::string> > dataList = reader.getData();

//	std::cout<<"dataList size ="<<dataList.size()<<std::endl;

  std::vector<float> depth_data1;
  for(int i=0;i<dataList.size();i++)
  {
    std::vector<std::string> vec = dataList[i];
    for (int j=0;j<vec.size();j++)
      // std::cout<<vec[j]<<","; 
      depth_data1.push_back( std::atof(vec[j].c_str()) );
  }
  std::cout<<"vec size :"<<depth_data1.size()<<"\n";

  for(int i=0;i<depth_data1.size();i++)
    std::cout<<depth_data1[i]<<",";
	return 0;

}
