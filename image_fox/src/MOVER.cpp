//using fuzzy control  
//ARRAYSIZE =1000; over will lead tosegmentation fault
#include "Mover.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Mover");

	robot robot;
    robot.nh;



	
    robot.change_map(4);//change map to 4F
    sleep(3);
    robot.Posepub(21.593,-16.039,0,0,0,1,-0.012,-3.117,1);//cirlab pose
    sleep(1);
    //ban area for reset position
    vector <double> ban_x={4,2.5,25,3.6};
    vector <double> ban_y={20,0,-16,-16};
    vector <double> ban_r={3,3,3,3};
    robot.Banlist(ban_x,ban_y,ban_r,"xr_4F_up");
    //  path planning
    vector <double> len ={18,16.5,2.8};
    vector <double> ang ={0,-90,0};

    robot.savetotxt("/home/nvidia/move_data/");
    robot.pathplanning(len,ang,"4F_lab",0.2,21.593,-16.039,180);
    robot.Custom_path("xr_4F_lab","yr_4F_lab",1);
	
	robot.stop();//Stop 


	return 0;
}
