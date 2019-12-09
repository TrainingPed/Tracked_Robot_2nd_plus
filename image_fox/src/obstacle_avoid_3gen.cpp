#include "obstacle_avoid_3gen.h"

using namespace std;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Obstacle_avoid_3gen");

    /*
    *Initialization : Read CSV and create average_depth array
    */
    // Read the non obstacle depth data from CSV File

    // CSVReader reader("/home/nvidia/Desktop/depth_data/depth_data0812.csv");
    CSVReader reader("/home/nvidia/Desktop/depth_data/depth_data0812.csv");
    std::vector<std::vector<std::string> > dataList = reader.getData();
    for(int i=0;i<dataList.size();i++)
    {
        std::vector<std::string> vec = dataList[i];
        for (int j=0;j<vec.size();j++)
            no_obs_depth.push_back( std::atof(vec[j].c_str()) );//convert into float and save as one dimension vector
    }
    avg_depth.resize(Row);   

    //create average depth vector 

    for(int y=0; y<Row; y++){
        avg_depth[y]=0;
        for(int x=0; x<Col ; x++){
                if( isnan(no_obs_depth[x+y*Col]))//process nan problem
                    no_obs_depth[x+y*Col]=0;
                else if(no_obs_depth[x+y*Col]>20)
                    no_obs_depth[x+y*Col]=20; //process infinite problem
                else if(no_obs_depth[x+y*Col]<0)
                    no_obs_depth[x+y*Col]=0;//process negative problem
                avg_depth[y]=avg_depth[y]+no_obs_depth[x+y*Col];
        }
        avg_depth[y]=avg_depth[y]/Col;
        std::cout<<"average depth "<<y<<":"<<avg_depth[y]<<"\n";
    }
    
    //start calc obstacle

	obstacle_avoid obs;
    
    // obs.thread_init();
    
    cout<<"start!"<<"\n";

    obs.main_work();
    // while(ros::ok())
    // { 
        
    //     ros::spinOnce();

    //     if (rgb_show)
    //         cv::imshow(OPENCV_WINDOW,img1);
    //     //show result
    //     if(cv::waitKey(33) == 27)//press esc to escape
    //     { 
    //         break;
    //     }
    //     if(img2_finsh)
    //     {
    //         obs.action_launcher();
    //     }
            
    //     cout<<"\n";
        
    //     //usleep(100000);//sleep 0.1 sec
    // }

    return 0;
}
