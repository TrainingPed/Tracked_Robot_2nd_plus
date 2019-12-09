#include "ros/ros.h"
#include <stdlib.h>
#include <stdio.h>
#include <iomanip>
#include <ctime>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "UpStairMode.h"
#include "DownStairMode.h"
#include "../obstacle_avoid_3gen.h"
#include "../Mover.h"

// Upstairmode + turn right 90 degree +Downstairmode version
class StairInformaion
{
public:
    std::string stair_type="";
    bool face_stair=false;
};
StairInformaion si;
class DemoMode
{
public:
    std::string NameSpace;
    DemoMode()
    {
        target1_sub=n.subscribe("/target1",1,&DemoMode::target1callback,this);
        laser_sub = n.subscribe("/scan", 1,&DemoMode::lasercallback,this);
        clearpath_pub =n.advertise<std_msgs::Int32>("pathclear",1);
        currentfloor_pub = n.advertise<std_msgs::String>("robot_stat",1);
    }
    int target1=0,target2=1,current_floor=0;
    string status;
    double laser_2,laser_32,laser_170,laser_260,laser_340,laser_481,laser_511;
    //mission function
    void Turn_180();
    void Turn_180_withR(int v1, int v2,int cnt);
    void Floor4_Floor5();
    void UpstairandDownstair1();
    void ConsecutiveUpstair1(std::string stat);
    void ConsecutiveDownstair1(std::string stat);
    void laser_UpStair();
    void laser_DownStair();
    void find_up_stair(int floor);
    void find_down_stair();
    void nav_direction();
    void test();
    void F4tolab();
    void F4fromlab();
    void F1_downstair();
    void F2_downstair();
    void F3_downstair();
    void F4_downstair();
    void F2_upstair();
    void F3_upstair();
    void F4_upstair();
    void F5_upstair();
    void infinity_upanddown(int init_floor);
    void patrol_mode(int init_floor);
    void full_navigation(int floor_number,string stat);
    void recursivepatrol(int init_floor);


    //ros subcriber callback
    void target1callback(const std_msgs::Int32::ConstPtr& msg);
    void lasercallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    //ros publisher function
    void clearpath(int clear);
    void info_callback(std::string info);
    ros::NodeHandle n;
private:
    ros::Subscriber target1_sub;
    ros::Subscriber laser_sub;
    ros::Publisher clearpath_pub;
    ros::Publisher currentfloor_pub;
};



int main(int argc, char **argv)
{
    int count=0;
    ros::init(argc, argv, "Demo_stairclimbing");

    UpStairMode USM;
    DownStairMode DSM;
    robot robot;
    DemoMode DM;
    USM.n;
    DSM.n;
    robot.nh;
    DM.n;

    printf("Demo start\n");
    ros::Rate r(2);

    std::string option;
    ros::param::get("/Demo_option",option);
    std::cout<<"Please choose the Demo Mode : \n"
             <<"(1) Laser based Upstair\n"
             <<"(2) Laser based Downstair\n"
             <<"(3) Consecutive Upstair (1)\n"
             <<"(4) Consecutive Downstair (1)\n"
             <<"(5) Upstair & Downstair\n"
             <<"(6) 4F find upstair navigation\n"
             <<"(7) 4F find downstair navigation\n"
             <<"(8) Double Consecutive Downstair\n"
             <<"(9) Double Consecutive Upstair\n"
             <<"(10) 4F to 5F with Navigation\n"
             <<"(test) test function\n"
             <<"(1F) 1F Navigation\n"
             <<"(2F) 2F Navigation\n"
             <<"(3F) 3F Navigation\n"
             <<"(4F) 4F Navigation\n"
             <<"(5F) 5F Navigation\n"
             <<"(INF) inifity up and down\n"
             <<"(Patrol) go for target floor\n"
             <<"(R-Patrol)listen on target floor and navigation\n";
    if(option=="")
        std::getline(std::cin, option);

    if(option == "1")
    {
        DM.laser_UpStair();
    }
    else if(option == "2"){
        DM.laser_DownStair();
    }
    else if(option == "3"){
        DM.ConsecutiveUpstair1("init");
    }
    else if(option == "4"){
        DM.ConsecutiveDownstair1("init");
    }
    else if(option == "5")
    {
        DM.UpstairandDownstair1();
    }
    else if(option == "6"){

        printf("Please enter initial floor!\n");
        int initfloor=0;
        std::cin>>initfloor;
        si.face_stair=false;
        si.stair_type="up";
        DM.find_up_stair(initfloor);
    }
    else if(option == "7"){
        si.face_stair=false;
        si.stair_type="down";
        DM.find_down_stair();
    }
    else if(option == "8")
    {
        DM.ConsecutiveDownstair1("init");
        si.face_stair=false;
        si.stair_type="down";
        DM.find_down_stair();
        DM.ConsecutiveDownstair1("4");
    }
    else if(option =="9")
    {
        DM.ConsecutiveUpstair1("init");
        si.face_stair=false;
        si.stair_type="up";
        DM.find_up_stair(4);
        DM.ConsecutiveUpstair1("4");
    }
    else if(option =="10")
    {
        robot.savetotxt("/home/nvidia/move_data/");
        DM.Floor4_Floor5();
    }
    else if(option =="test")
    {
        DM.test();
    }
    else if(option =="1F")
    {
        DM.F1_downstair();
    }
    else if(option =="2F")
    {
        string mode;
        printf("Welcome to 2F Navigation Mode!\n");
        std::cout<<"Please choose the Demo Mode : \n"
                 <<"(1) Navigate full floor from Stair1(upstair)\n"
                 <<"(2) Navigate full floor from Stair1(downstair)\n";
        std::getline(std::cin,mode );
        if(mode ==  "1")
            DM.F2_upstair();
        else if(mode ==  "2")
            DM.F2_downstair();
    }
    else if(option =="3F")
    {
        string mode;
        printf("Welcome to 3F Navigation Mode!\n");
        std::cout<<"Please choose the Demo Mode : \n"
                 <<"(1) Navigate full floor from Stair1(upstair)\n"
                 <<"(2) Navigate full floor from Stair1(downstair)\n";
        std::getline(std::cin,mode );
        if(mode ==  "1")
            DM.F3_upstair();
        else if(mode ==  "2")
            DM.F3_downstair();
    }
    else if(option =="4F")
    {
        string mode;
        printf("Welcome to 4F Navigation Mode!\n");
        std::cout<<"Please choose the Demo Mode : \n"
                 <<"(1) Navigate from Stair1 to CIRLAB\n"
                 <<"(2) Navigate from CIRLAB to Stair1\n"
                 <<"(3) Navigate full floor from Stair1(upstair)\n"
                 <<"(4) Navigate full floor from Stair1(downstair)\n";
        std::getline(std::cin,mode );
        if(mode ==  "1")
            DM.F4tolab();
        else if(mode ==  "2")
            DM.F4fromlab();
        else if(mode ==  "3")
            DM.F4_upstair();
        else if(mode ==  "4")
            DM.F4_downstair();
    }
    else if(option =="5F")
    {
        DM.F5_upstair();
    }
    else if(option =="INF")
    {
        string init_floor;
        printf("Welcome to Infinity Up&Down Mode!\n");
        printf("Please enter initial floor!\n");
        int initfloor=0;
        std::cin>>initfloor;
        printf("Please enter target floor!\n");
        std::cin>>DM.target2;
        string stair_op;
        printf("Does the robot face the stair?(Y/N)\n");
        std::cin>>stair_op;
        if(stair_op=="Y")
            si.face_stair=true;
        else if(stair_op=="N")
            si.face_stair=false;
        else
            initfloor=0;
        printf("What stair type it is?(up/down)\n");
        std::cin>>si.stair_type;
        cout<<"init floor = "<<initfloor<<"\n";
        if((initfloor>0) && (initfloor<6)){
            cout<<"start!\n";
            DM.infinity_upanddown(initfloor);
        }
    }
    else if(option == "Patrol")
    {
        string init_floor;
        printf("Welcome to Patrol Mode!\n");
        printf("Please enter initial floor!\n");
        int initfloor=0;
        std::cin>>initfloor;
        printf("Please enter target floor!\n");
        std::cin>>DM.target1;
        cout<<"init floor = "<<initfloor<<"\n";
        string stair_op;
        printf("Does the robot face the stair?(Y/N)\n");
        std::cin>>stair_op;
        if(stair_op=="Y")
            si.face_stair=true;
        else if(stair_op=="N")
            si.face_stair=false;
        else
            initfloor=0;
        printf("What stair type it is ?(up/down)\n");
        std::cin>>si.stair_type;
        if((initfloor>0) && (initfloor<6)){//only allow 1~5F
            cout<<"start!\n";
            DM.patrol_mode(initfloor);
        }
    }
    else if(option == "R-Patrol")
    {
        string init_floor;
        printf("Welcome to Recursive Patrol Mode!\n");
        printf("Do you want auto-patrol?(Y/N)\n");
        std::string auto_patrol;
        std::cin>>auto_patrol;
        if(auto_patrol=="Y")
            ros::param::set("/inf_on",1);
        else if(auto_patrol=="N")
            ros::param::set("/inf_on",0);
        int initfloor;
        printf("Please enter initial floor!\n");
        std::cin>>initfloor;
        cout<<"init floor = "<<initfloor<<"\n";
        string stair_op;
        printf("Does the robot face the stair?(Y/N)\n");
        std::cin>>stair_op;
        if(stair_op=="Y")
            si.face_stair=true;
        else if(stair_op=="N")
            si.face_stair=false;
        else
            initfloor=0;
        printf("What stair type it is ?(up/down)\n");
        std::cin>>si.stair_type;
        if((initfloor>0) && (initfloor<6)){//only allow 1~5F
            cout<<"start!\n";
            DM.recursivepatrol(initfloor);
        }
    }
    ros::param::set("/Demo_option","");
    printf("Demo End ...\n");
    return 0;

}
void DemoMode::test()
{
//    robot().turn_any_angle(-90,5,0.5);
//    UpStairMode().wall_alignment_r(0.02,0.04);
//    UpStairMode().legMA_feedback(-486000,-426000);

//    UpStairMode().stair_alignment(0.02);
    DemoMode::clearpath(1);
    DemoMode::clearpath(0);
    int map_no=1;
    n.getParam("map_no", map_no);
    robot().change_map(map_no);
    sleep(3);
    robot().Posepub(0.430, -3.831, 0.000,0.000, 0.000, 0.018, 1.000,0.035,1);

    vector<double> len;
    vector<double> ang;
    vector<double> pose;
    if( !n.getParam("len", len) )
        ROS_ERROR("Failed to get len parameter from server.");
    else
        std::cout<<"suceess reading len... \n";
    if( !n.getParam("ang", ang) )
        ROS_ERROR("Failed to get ang parameter from server.");
    else
        std::cout<<"suceess reading ang... \n";

    n.getParam("pose", pose);

    robot().pathplanning(len,ang,"prestair",0.3,pose[0],pose[1],pose[2]);

    vector <double> xr1;
    vector <double> yr1;
    if( !n.getParam("xr_prestair", xr1) )
        ROS_ERROR("Failed to get xr parameter from server.");
    else
        std::cout<<"suceess reading xr1... \n";
    if( !n.getParam("yr_prestair", yr1) )
        ROS_ERROR("Failed to get xr parameter from server.");
    else
        std::cout<<"suceess reading yr1... \n";
    robot().Pathpub(xr1,yr1);



}
void DemoMode::info_callback(std::string info)
{
    std_msgs::String msg;
    msg.data = info;
    currentfloor_pub.publish(msg);
    ros::spinOnce();
}
void DemoMode::clearpath(int clear)
{
    std_msgs::Int32 msg;
    msg.data = clear;
    clearpath_pub.publish(msg);
    ros::spinOnce();
}
void DemoMode::target1callback(const std_msgs::Int32::ConstPtr &msg)
{
    target1=msg->data;
}
void DemoMode::lasercallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    laser_2 = msg->ranges[2];
    laser_32 = msg->ranges[32];
    laser_170 = msg->ranges[180];
    laser_340 = msg->ranges[355];
    laser_260 = msg->ranges[260];
    laser_481 = msg->ranges[481];
    laser_511 = msg->ranges[511];
}

void DemoMode::infinity_upanddown(int init_floor)
{
    current_floor =init_floor;

    while(target1 == 0)
    {
        std::string str = std::to_string(current_floor);
        DemoMode::info_callback(str+" auto-climbing");
        if(target1 != 0 )
            break;
        if(current_floor == 5)//
        {

            find_down_stair();//last direction of robot
            ConsecutiveDownstair1("5");
            target2=1;
            current_floor--;
            status ="down";
            si.face_stair=false;
            si.stair_type="down";
        }
        else if(current_floor == 1)
        {
            find_up_stair(current_floor);
            ConsecutiveUpstair1("1");
            target2=5;
            current_floor++;
            status ="up";
            si.face_stair=false;
            si.stair_type="up";
        }
        else if(current_floor < target2)
        {
            std::string floor1=std::to_string(current_floor);
            find_up_stair(current_floor);
            ConsecutiveUpstair1(floor1);
            target2=5;
            current_floor++;
            status ="up";
            si.face_stair=false;
            si.stair_type="up";
        }
        else
        {
            std::string floor1=std::to_string(current_floor);
            find_down_stair();
            ConsecutiveDownstair1(floor1);
            target2=1;
            current_floor--;
            status ="down";
            si.face_stair=false;
            si.stair_type="down";
        }
        ros::spinOnce();
    }
    robot().stop();
}
void DemoMode::patrol_mode(int init_floor)
{
    current_floor =init_floor;
    while(target1 != current_floor)
    {
            std::string str = std::to_string(current_floor);
            std::string str2 = std::to_string(target1);
            DemoMode::info_callback(str+" climb stair to"+str2+"F");
            if(target1 == 0 )
                break;
            if(current_floor == 5)//
            {
                find_down_stair();//last direction of robot
                ConsecutiveDownstair1("5");
                current_floor--;
                status ="down";
                si.face_stair=false;
                si.stair_type="down";
            }
            else if(current_floor == 1)
            {

                find_up_stair(current_floor);
                ConsecutiveUpstair1("1");
                current_floor++;
                status ="up";
                si.face_stair=false;
                si.stair_type="up";
            }
            else if(current_floor < target1)
            {
                std::string floor1=std::to_string(current_floor);
                find_up_stair(current_floor);
                ConsecutiveUpstair1(floor1);
                current_floor++;
                status ="up";
                si.face_stair=false;
                si.stair_type="up";
            }
            else
            {
                std::string floor1=std::to_string(current_floor);
                find_down_stair();
                ConsecutiveDownstair1(floor1);
                current_floor--;
                status ="down";
                si.face_stair=false;
                si.stair_type="down";
            }
            ros::spinOnce();
    }
//    target1 = 0;

}
void DemoMode::full_navigation(int floor_number, string stat)
{
    ros::Rate r(2);
    std::string str = std::to_string(floor_number);
    DemoMode::info_callback(str+" navigate "+ str+"F");
    DemoMode::nav_direction();
    robot().reset_fusion("zero");

    //lift legs before start navigation
    DownStairMode().dynamixel_motor(0,0);
    sleep(1);
    UpStairMode().legspeed(2500,2500);
    sleep(1);
    UpStairMode().legMA_feedback(-486000,-426000);

    switch(floor_number)
    {
    case 1:
        DemoMode::F1_downstair();
        UpStairMode().stair_alignment(0.02);
        si.face_stair=true;
        si.stair_type="up";
        break;
    case 2:
        if(stat=="up"){
            UpStairMode().wall_alignment_r(0.02,0.04);
            DemoMode::F2_upstair();
            si.face_stair=true;
            si.stair_type="down";
        }
        else if(stat =="down"){
            DemoMode::F2_downstair();
            UpStairMode().stair_alignment(0.02);
            si.face_stair=true;
            si.stair_type="up";
        }
        break;
    case 3:
        if(stat=="up"){
            UpStairMode().wall_alignment_r(0.02,0.04);
            DemoMode::F3_upstair();
            si.face_stair=true;
            si.stair_type="down";
        }
        else if(stat =="down"){
            DemoMode::F3_downstair();
            UpStairMode().stair_alignment(0.02);
            si.face_stair=true;
            si.stair_type="up";
        }
        break;
    case 4:
        if(stat=="up"){
            UpStairMode().wall_alignment_r(0.02,0.04);
            DemoMode::F4_upstair();
            si.face_stair=true;
            si.stair_type="down";
        }
        else if(stat =="down"){
            DemoMode::F4_downstair();
            UpStairMode().stair_alignment(0.02);
            si.face_stair=true;
            si.stair_type="up";
        }
        break;
    case 5:
        UpStairMode().wall_alignment_r(0.02,0.04);
        DemoMode::F5_upstair();
        si.face_stair=true;
        si.stair_type="down";
        break;
    default:
        break;

    }
    DownStairMode().legMA(0,0);
    sleep(8);
    //caliberate the motor by servo off
    DownStairMode().motor_switch(1,0);
    r.sleep();
    DownStairMode().motor_switch(4,0);
    r.sleep();
    sleep(1);
    DownStairMode().motor_switch(1,1);
    r.sleep();
    DownStairMode().motor_switch(4,1);
    r.sleep();

    std::cout<<"Stop Navigation\n";

}
void DemoMode::recursivepatrol(int init_floor)
{
    ros::Rate r(2);
    int inf_on=1;
    current_floor = init_floor;
    while(ros::ok())
    {
        ros::param::get("inf_on",inf_on);
        if(target1 == 0 && inf_on)//if user hadn't assigned target floor
            DemoMode::infinity_upanddown(current_floor);//keep climbing up and down on the stair
        else if(target1 == 0)
        {
            std::string str = std::to_string(current_floor);
            DemoMode::info_callback(str+"wait for command");
            r.sleep();
        }
        else if(target1 != current_floor)//if user had assigned target and robot is not at the same floor
            DemoMode::patrol_mode(current_floor);//go for target floor
        else if(target1 == current_floor)//if user had assigned target and robot is at the same floor
        {
            DemoMode::full_navigation(current_floor,si.stair_type);//select the map and navigate full floor
            target1=0;//reset target

        }
        ros::spinOnce();
        r.sleep();
        std::cout<<"target1="<<target1<<"\n";
    }
}
void DemoMode::F4tolab()
{
    DemoMode::clearpath(1);
    robot().change_map(4);
    sleep(3);
    DemoMode::clearpath(0);
    //ban area for reset position
    vector <double> ban_x={4,2.5,25,3.6};
    vector <double> ban_y={20,0,-16,-16};
    vector <double> ban_r={3,3,3,3};
   robot().Banlist(ban_x,ban_y,ban_r,"xr_4F_lab");
    //  path planning
    vector <double> len ={3.8,16.5,18};
    vector <double> ang ={0,-90,0};
    robot().savetotxt("/home/nvidia/move_data/");
    robot().pathplanning(len,ang,"4F_lab",0.2,0,0,0);
    robot().Custom_path("xr_4F_lab","yr_4F_lab",1);
    robot().stop();
}
void DemoMode::F4fromlab()
{
    DemoMode::clearpath(1);
    robot().change_map(4);
    sleep(3);
    DemoMode::clearpath(0);
    robot().Posepub(21.593,-16.039,0,0,0,1,-0.012,-3.117,1);//cirlab pose
    sleep(1);
    //ban area for reset position
    vector <double> ban_x={4,2.5,25,3.6};
    vector <double> ban_y={20,0,-16,-16};
    vector <double> ban_r={3,3,3,3};
    robot().Banlist(ban_x,ban_y,ban_r,"xr_4F_lab");
    //  path planning
    vector <double> len ={18,16.5,2.8};
    vector <double> ang ={0,-90,0};
    robot().savetotxt("/home/nvidia/move_data/");

    robot().pathplanning(len,ang,"4F_lab",0.2,21.593,-16.039,180);
    robot().Custom_path("xr_4F_lab","yr_4F_lab",1);
    robot().stop();
}
void DemoMode::F1_downstair()
{
    DemoMode::clearpath(1);
    robot().change_map(1);
    sleep(5);
    DemoMode::clearpath(0);
    vector <double> len ={2.46,16.6,20,20,16.6,2.96};
    vector <double> ang ={0,-90,0,-180,90,-180};
    robot().reset_fusion("zero");
    robot().savetotxt("/home/nvidia/move_data/");
    robot().pathplanning(len,ang,"1F_down",0.3,0,0,0);
    robot().Custom_path("xr_1F_down","yr_1F_down",1);
    robot().stop();
}

void DemoMode::F2_downstair()
{
    DemoMode::clearpath(1);
    robot().change_map(2);
    sleep(5);
    DemoMode::clearpath(0);


    //set ban area of reset amcl position
    vector <double> ban_x={1.7,3.4,7.5,24,0.5,0.0};
    vector <double> ban_y={-1.1,-16,-16,-16,18,22};
    vector <double> ban_r={5,3,3,3,3,3};
    robot().Banlist(ban_x,ban_y,ban_r,"xr_2F_down");

    vector <double> len ={3.7,16.5,33,33,16,3,27.9,27.4,0.7};
    vector <double> ang ={0,-90,1,-179,90,-180,91,-89,-180};
    robot().reset_fusion("zero");
    robot().savetotxt("/home/nvidia/move_data/");
    robot().pathplanning(len,ang,"2F_down",0.3,0,0,0);
    robot().Custom_path("xr_2F_down","yr_2F_down",1);
    robot().stop();
}
void DemoMode::F3_downstair()
{
    DemoMode::clearpath(1);
    robot().change_map(3);
    sleep(5);
    DemoMode::clearpath(0);

    //set ban area of reset amcl position
    vector <double> ban_x={0.9,3.9,7.72,4.4,0.5,0.3,41.2};
    vector <double> ban_y={-1,-15.8,-15.9,-15.3,15.6,28.3,-14.4};
    vector <double> ban_r={5,3,3,3,3,5,3};
    robot().Banlist(ban_x,ban_y,ban_r,"xr_3F_down");

//    vector <double> len ={3.9,16.1,43,43,16.1,3.6,27.7,27.7,0.3};
//    vector <double> ang ={0,-90,2,182,90,-180,90,-90,-180};

    vector <double> len ={3.9,16.1,30,3,3,30,16.1,3.6,27.7,27.7,0.6};
    vector <double> ang ={0,-90,3,0,180,183,90,-180,90,-90,-180};
    robot().reset_fusion("zero");
    robot().savetotxt("/home/nvidia/move_data/");
    robot().pathplanning(len,ang,"3F_down",0.3,0,0,0);
    robot().Custom_path("xr_3F_down","yr_3F_down",1);
    robot().stop();
}
void DemoMode::F4_downstair()
{
    DemoMode::clearpath(1);
    robot().change_map(4);
    sleep(5);
    DemoMode::clearpath(0);

    //set ban area of reset amcl position
    vector <double> ban_x={2.5,25,3.6};
    vector <double> ban_y={0,-16,-16};
    vector <double> ban_r={3,3,3};
    robot().Banlist(ban_x,ban_y,ban_r,"xr_4F_down");
    robot().Banlist(ban_x,ban_y,ban_r,"xr_4F_down");

    robot().savetotxt("/home/nvidia/move_data/");
    robot().reset_fusion("zero");
    //path planning
//    vector <double> len ={4,16.1,32,32.5,16.1,22.9,22.9,4};
//    vector <double> ang ={0,-90,0,-180,90,89,-91,-180};
    vector <double> len ={3.5,16.1,32.5,32.5,16.1,22.9,22.9,4};
    vector <double> ang ={0,-90,0,-180,90,89,-91,-180};
    robot().pathplanning(len,ang,"4F_down",0.3,0,0,0);
    robot().Custom_path("xr_4F_down","yr_4F_down",1);
    robot().stop();
}
void DemoMode::F2_upstair()
{
    DemoMode::clearpath(1);
    robot().change_map(2);
    sleep(3);
    DemoMode::clearpath(0);
    robot().Posepub(0,-1.9,0,0,-0.015,1,1,0,1);//cirlab pose
    sleep(3);

    //set ban area of reset amcl position
    vector <double> ban_x={1.7,3.4,7.5,24,0.5,0.0};
    vector <double> ban_y={-1.1,-16,-16,-16,18,22};
    vector <double> ban_r={5,3,3,3,3,3};
    robot().Banlist(ban_x,ban_y,ban_r,"xr_2F_up");

    vector <double> len ={3.7,14,33,33,16.2,3,27.9,30,0.7};
    vector <double> ang ={0,-90,0,-180,90,-180,91,-89,-180};
    robot().reset_fusion("zero");
    robot().savetotxt("/home/nvidia/move_data/");
    robot().pathplanning(len,ang,"2F_up",0.3,0,-1.9,0);
    robot().Custom_path("xr_2F_up","yr_2F_up",1);
    robot().stop();
}
void DemoMode::F3_upstair()
{
    DemoMode::clearpath(1);
    robot().change_map(3);
    sleep(3);
    DemoMode::clearpath(0);
    robot().Posepub(0,-1.83,0,0,0,0.003,1,0.006,1);//cirlab pose
    sleep(5);
    //set ban area of reset amcl position
    vector <double> ban_x={0.9,3.9,24.4,0.5,0.3,41.2};
    vector <double> ban_y={-1,-15.8,-15.3,15.6,28.3,-14.4};
    vector <double> ban_r={5,3,3,3,3,3};
    robot().Banlist(ban_x,ban_y,ban_r,"xr_3F_up");

    vector <double> len ={3.9,14.2,34,34,16.1,3.6,27.7,29.5,0.6};
    vector <double> ang ={0,-90,2,182,90,-180,90,-90,-180};
    robot().reset_fusion("zero");
    robot().savetotxt("/home/nvidia/move_data/");
    robot().pathplanning(len,ang,"3F_up",0.3,0,-1.83,0);
    robot().Custom_path("xr_3F_up","yr_3F_up",1);
    robot().stop();
}
void DemoMode::F4_upstair()
{
    DemoMode::clearpath(1);
    robot().change_map(4);
    sleep(3);
    DemoMode::clearpath(0);
    //set initial pose
    robot().Posepub(-0.341,-2.006,0,0,0,-0.017,1,-0.033,1);
    sleep(5);
    //set ban area of reset amcl position
    vector <double> ban_x={4,2.5,25,3.6};
    vector <double> ban_y={20,0,-16,-16};
    vector <double> ban_r={3,3,4,3};
    robot().Banlist(ban_x,ban_y,ban_r,"xr_4F_up");
    robot().reset_fusion("zero");
    //save log
    robot().savetotxt("/home/nvidia/move_data/");
    //plan a path with path length and angle
    vector <double> len ={4,14.5,32,32,16.5,23,25,4};
    vector <double> ang ={0,-90,0,-180,90,89,-91,-180};
    robot().pathplanning(len,ang,"4F_up",0.3,-0.341,-2.006,0);
    //start path tracking mission
    robot().Custom_path("xr_4F_up","yr_4F_up",1);
    robot().stop();
}
void DemoMode::F5_upstair()
{
    DemoMode::clearpath(1);
    robot().change_map(5);
    sleep(3);
    DemoMode::clearpath(0);


    //set ban area of reset amcl position
    vector <double> ban_x={0.4,4.1,8.5,24.3,0.9,29.3};//4.3 (box)
    vector <double> ban_y={-1,-16,-16,-15.6,17.9,-15.1};//-5.6 (box)
    vector <double> ban_r={5,3,3,3,3,3};
    robot().Banlist(ban_x,ban_y,ban_r,"xr_5F_up");

//    robot().Posepub(-0.182,-1.77,0,0,0,0.002,1,-0.004,1);//cirlab pose
    robot().Posepub(0.430, -3.831, 0.000,0.000, 0.000, 0.018, 1.000,0.035,1);
    sleep(3);
    vector <double> len ={0.848,3.1,15.1,21,21,16.4,2.8,20.3,22.5,1};
    vector <double> ang ={45,0,-90,0,-180,90,-180,90,-90,-180};
    robot().reset_fusion("zero");
    robot().savetotxt("/home/nvidia/move_data/");
    robot().pathplanning(len,ang,"5F_up",0.3,0.430, -3.831,0.035);
    robot().Custom_path("xr_5F_up","yr_5F_up",1);
    robot().stop();
}
void DemoMode::find_down_stair()
{
    if(si.face_stair==false && si.stair_type=="down")
    {
        //Lift legs
        DownStairMode().dynamixel_motor(0,0);
        sleep(1);
        UpStairMode().legspeed(2500,2500);
        sleep(1);
        UpStairMode().legMA(-436000,-426000);
        sleep(6);
        //turn right 90

        robot().resetodom();
        robot().reset_fusion("zero");
        //go straight 1 m
        vector <double> len={1};
        vector <double> ang={0};
        robot().pathplanning(len,ang,"prestair",0.05,0,0,0);
        robot().Custom_linear_path("xr_prestair","yr_prestair",0);

        robot().turn_any_angle(-90,5,0.5);
        UpStairMode().wall_alignment_r(0.02,0.04);
//        robot().resetodom();
        //approach wall and stop with laser
        UpStairMode().wall_approach_laser(0.08,0.04,0.8,0.75,1);
        //turn right 90
        robot().turn_any_angle(-90,5,0.5);
        sleep(1);
        //alignment with left wall
        UpStairMode().wall_alignment_l(0.03,0.04);
    }
    else if(si.face_stair == false&&si.stair_type == "up")//robot just go upstair and want to go down again
    {
        //Lift legs
        DownStairMode().dynamixel_motor(0,0);
        sleep(1);
        UpStairMode().legspeed(2500,2500);
        sleep(1);
        UpStairMode().legMA(-436000,-426000);
        sleep(6);
        //turn right 180
        robot().turn_any_angle(180,10,0.5);
        //        Turn_180();
//        UpStairMode().wall_alignment_l(0.03,0.04);
    }
    else if(si.face_stair== true && si.stair_type =="down")
    {
        printf("robot first time go downstair!\n");
        //Lift legs
        DownStairMode().dynamixel_motor(0,0);
        sleep(1);
        UpStairMode().legspeed(2500,2500);
        sleep(1);
        UpStairMode().legMA(-436000,-426000);
        sleep(6);
        UpStairMode().wall_alignment_l(0.03,0.04);

    }
    else if(si.face_stair== true && si.stair_type =="up")
    {
        printf("robot first time go downstair!\n");
        //Lift legs
        DownStairMode().dynamixel_motor(0,0);
        sleep(1);
        UpStairMode().legspeed(2500,2500);
        sleep(1);
        UpStairMode().legMA(-436000,-426000);
        sleep(6);
        //turn right 180
        robot().turn_any_angle(180,10,0.5);

        //go straight 1 m
        robot().resetodom();
        robot().reset_fusion("zero");
        vector <double> len={0.5};
        vector <double> ang={0};
        robot().pathplanning(len,ang,"prestair",0.05,0,0,0);
        robot().Custom_linear_path("xr_prestair","yr_prestair",0);
        //turn right 90
        robot().turn_any_angle(-90,5,0.5);
        UpStairMode().wall_alignment_r(0.02,0.04);
//        sleep(1);
//        robot().resetodom();
        //approach wall and stop with laser
        UpStairMode().wall_approach_laser(0.08,0.04,0.8,0.75,1);
        sleep(1);
        //turn right 90
        robot().turn_any_angle(-90,5,0.5);
        //alignment with left wall
        UpStairMode().wall_alignment_l(0.03,0.04);

    }
}
void DemoMode::find_up_stair(int current_floor)
{
    double total_d[5]={3,3,3,3,3};
    ros::Rate r(2);
    if(si.face_stair == false && si.stair_type=="up")
    {
        //Lift legs
        DownStairMode().dynamixel_motor(0,0);
        sleep(1);
        UpStairMode().legspeed(2500,2500);
        sleep(1);
        UpStairMode().legMA_feedback(-486000,-426000);
        //change map and reset odom
        UpStairMode().wall_alignment_r(0.03,0.04);
        double d2=0,d1=0;//distance to the next stair

        robot().resetodom();
        robot().reset_fusion("zero");
        //go straight 1 m
        vector <double> len={1};
        vector <double> ang={0};
        robot().pathplanning(len,ang,"prestair",0.05,0,0,0);
        robot().Custom_linear_path("xr_prestair","yr_prestair",0);

        // alignmen to the wall and measure d1
        UpStairMode().wall_alignment_r(0.03,0.04);
        r.sleep();
        d1 = laser_2;//distance to the wall

        d2 = total_d[current_floor-1] - d1;


        //turn right 90
        robot().turn_any_angle(90,5,0.5);

        robot().resetodom();

        robot().reset_fusion("zero");

        printf("go straight for %f m\n\n",d2);
//            sleep(1);
        vector <double> len2;
        len2.push_back(d2);
        vector <double> ang2={0};

        robot().pathplanning(len2,ang2,"prestair2",0.05,0,0,0);
        robot().Custom_linear_path("xr_prestair2","yr_prestair2",0);
        r.sleep();
        //turn right 90
        robot().turn_any_angle(90,5,0.55);
    }
    else if(si.face_stair==false && si.stair_type == "down")
    {
        DownStairMode().dynamixel_motor(0,0);
        sleep(1);
        UpStairMode().legspeed(2500,2500);
        sleep(1);
        UpStairMode().legMA_feedback(-486000,-426000);
        robot().turn_any_angle(180,10,0.5);
        /*Turn_180*/
    }
    else if(si.face_stair==true && si.stair_type=="up")
    {
        DownStairMode().dynamixel_motor(0,0);
        sleep(1);
        UpStairMode().legspeed(2500,2500);
        sleep(1);
        UpStairMode().legMA_feedback(-486000,-426000);
        printf("lifted up the legs!");
    }
    else if(si.face_stair==true && si.stair_type=="down")
    {

        DownStairMode().dynamixel_motor(0,0);
        sleep(1);
        UpStairMode().legspeed(2500,2500);
        sleep(1);
        UpStairMode().legMA_feedback(-486000,-426000);
        printf("lifted up the legs!");
        robot().turn_any_angle(180,10,0.5);
        UpStairMode().wall_alignment_r(0.03,0.04);
        double d2=0,d1=0;//distance to the next stair

        robot().resetodom();
        robot().reset_fusion("zero");
        //go straight 1 m
        vector <double> len={1};
        vector <double> ang={0};
        robot().pathplanning(len,ang,"prestair",0.05,0,0,0);
        robot().Custom_linear_path("xr_prestair","yr_prestair",0);

        // alignmen to the wall and measure d1
        UpStairMode().wall_alignment_r(0.03,0.04);
        r.sleep();
        d1 = laser_2;//distance to the wall

        d2 = total_d[current_floor-1] - d1;


        //turn right 90
        robot().turn_any_angle(90,5,0.5);

        robot().resetodom();

        robot().reset_fusion("zero");

        printf("go straight for %f m\n\n",d2);
//            sleep(1);
        vector <double> len2;
        len2.push_back(d2);
        vector <double> ang2={0};

        robot().pathplanning(len2,ang2,"prestair2",0.05,0,0,0);
        robot().Custom_linear_path("xr_prestair2","yr_prestair2",0);
        r.sleep();
        //turn right 90
        robot().turn_any_angle(90,5,0.55);
    }



}
void DemoMode::nav_direction()
{
    if (si.face_stair == true)
    {
        robot().turn_any_angle(180,10,0.5);
        if(si.stair_type == "up")
        {
            si.face_stair=false;
            si.stair_type="down";
        }
        else if(si.stair_type == "down")
        {
            si.face_stair=false;
            si.stair_type="up";
        }
    }


}

void DemoMode::Turn_180_withR(int v1, int v2,int cnt)//v1 =left wheel speed,v2=right wheel speed
{
    ros::Rate r(2);
    UpStairMode().robotspeed(v1,v2);
    r.sleep();
    UpStairMode().robotmotion("Forward");
    r.sleep();
    for (int i=1;i<=cnt;i++)
    {
        sleep(1);
        std::cout<<"sleep for "<<i<<"sec\n";
    }
    UpStairMode().robotmotion("Stop");
}
void DemoMode::Turn_180()
{
    ros::Rate r(2);
    UpStairMode().robotspeed(700,700);
    r.sleep();
    UpStairMode().robotmotion("Forward");
    sleep(2);
    UpStairMode().robotmotion("Left");
    r.sleep();
    for (int i=1;i<=11;i++)
    {
        sleep(1);
        std::cout<<"sleep for "<<i<<"sec\n";
    }
    UpStairMode().robotmotion("Stop");
}
void DemoMode::laser_UpStair()
{
    std::string dir;
    printf("Welcome to laser Up Stair Mode!\n");
    printf("Please enter Left or Right!\n");
    std::getline(std::cin, dir);

    DownStairMode().leghome(0,0);
    sleep(1);
    DownStairMode().dynamixel_motor(0,0);
    sleep(1);
    UpStairMode().legspeed(2500,2500);
    sleep(1);
    UpStairMode().legMA(-436000,-426000);
    sleep(6);

    UpStairMode().mode_23(0.05,0.5,0.45,0.8,false);
    UpStairMode().mode_3();
    UpStairMode().mode_41(dir,10);

    DownStairMode().motor_switch(1,0);
    usleep(500000);
    DownStairMode().motor_switch(4,0);
    usleep(500000);
    sleep(1);
    DownStairMode().motor_switch(1,1);
    usleep(500000);
    DownStairMode().motor_switch(4,1);
    usleep(500000);

}
void DemoMode::laser_DownStair()
{
    std::string dir;
    printf("Welcome to laser Down Stair Mode!\n");
    printf("Please enter Left or Right!\n");
    std::getline(std::cin, dir);
    DownStairMode().leghome(0,0);
    sleep(1);
    DownStairMode().dynamixel_motor(0,0);
    sleep(1);
    DownStairMode().mode_6();
    DownStairMode().mode_71(dir);

    DownStairMode().motor_switch(1,0);
    usleep(500000);
    DownStairMode().motor_switch(4,0);
    usleep(500000);
    sleep(1);
    DownStairMode().motor_switch(1,1);
    usleep(500000);
    DownStairMode().motor_switch(4,1);
    usleep(500000);


}
void DemoMode::ConsecutiveDownstair1(std::string stat)
{
    ros::Rate r(2);

    printf("Consecutive Downstair (1) Start!\n");
    if(stat == "init")
    {
        //Initial motors
        DownStairMode().leghome(0,0);
        sleep(1);
        DownStairMode().dynamixel_motor(0,0);
        sleep(1);
    }
    else
    {
        UpStairMode().wall_alignment_l(0.02,0.04);
        r.sleep();
        DownStairMode().mode_51(0,0.05,0.08,0.04,200);//100
    }

    //Down Stair
    DownStairMode().mode_6();
    DownStairMode().mode_71("Left");
//    DownStairMode().mode_7();

    //caliberate the motor by servo off
    DownStairMode().motor_switch(1,0);
    usleep(500000);
    DownStairMode().motor_switch(4,0);
    usleep(500000);
    sleep(1);
    DownStairMode().motor_switch(1,1);
    usleep(500000);
    DownStairMode().motor_switch(4,1);
    usleep(500000);
    DownStairMode().leghome(0,0);
    sleep(1);
    DownStairMode().dynamixel_motor(0,0);
    sleep(1);
    //===Wall following (left)====
    UpStairMode().legspeed(2000,2000);
    sleep(1);

    UpStairMode().legMA(-486000,-426000);
    sleep(9);

    UpStairMode().wall_alignment_l(0.02,0.04);
    r.sleep();
    UpStairMode().wall_following_l(0,0.05,0.1,0.04,0.8,0.75,1);
    r.sleep();
    UpStairMode().robotspeed(350,350);
    r.sleep();
    //===Turn right 90 degree===
    UpStairMode().robotmotion("Right");
    r.sleep();
    for (int i=1;i<10;i++)//i<10
    {
        sleep(1);
        std::cout<<"sleep for "<<i<<"sec\n";
    }
    UpStairMode().robotmotion("Stop");
    r.sleep();
    UpStairMode().wall_alignment_l(0.02,0.04);
    r.sleep();
    //===Wall following (left)====
    UpStairMode().wall_following_l(0,0.05,0.1,0.04,0.65,0.6,0.8);
    r.sleep();
    //===Turn rught 90 degree===
    UpStairMode().robotspeed(350,350);
    r.sleep();
    UpStairMode().robotmotion("Right");
    r.sleep();
    for (int i=1;i<10;i++)
    {
        sleep(1);
        std::cout<<"sleep for "<<i<<"sec\n";
    }
    UpStairMode().robotmotion("Stop");
    r.sleep();
    UpStairMode().wall_alignment_l(0.02,0.04);
    r.sleep();
        //2nd Down Stair
    DownStairMode().mode_51(0,0.05,0.08,0.04,200);//100
    DownStairMode().mode_6();
    DownStairMode().mode_71("Left");
    //    DownStairMode().mode_7();

//    caliberate the motor by servo off
    DownStairMode().motor_switch(1,0);
    usleep(500000);
    DownStairMode().motor_switch(4,0);
    usleep(500000);
    sleep(1);
    DownStairMode().motor_switch(1,1);
    usleep(500000);
    DownStairMode().motor_switch(4,1);
    usleep(500000);
    DownStairMode().leghome(0,0);
    sleep(1);
    DownStairMode().dynamixel_motor(0,0);
    sleep(1);
}
void DemoMode::ConsecutiveUpstair1(std::string stat)
{
    ros::Rate r(2);

    printf("Consecutive Upstair (1) Start!\n");

    if(stat == "init")
    {
        //===Initialize leg and dynamixel===
        DownStairMode().leghome(0,0);
        sleep(1);
        DownStairMode().dynamixel_motor(0,0);
        sleep(1);
        UpStairMode().legspeed(2500,2500);
        sleep(1);
        UpStairMode().legMA_feedback(-486000,-426000);
    }

    //===Start climbing===
    UpStairMode().mode_23(0.1,0.5,0.45,0.8,false);
    UpStairMode().mode_3();
    UpStairMode().mode_41("Right",10);
//    UpStairMode().mode_4();

    //caliberate the motor by servo off
    DownStairMode().motor_switch(1,0);
    usleep(500000);
    DownStairMode().motor_switch(4,0);
    usleep(500000);
    sleep(1);
    DownStairMode().motor_switch(1,1);
    usleep(500000);
    DownStairMode().motor_switch(4,1);
    usleep(500000);
    DownStairMode().leghome(0,0);
    sleep(1);
    DownStairMode().dynamixel_motor(0,0);
    sleep(1);
    //===Wall following (right)====
    UpStairMode().legspeed(2000,2000);
    sleep(1);
    UpStairMode().legMA_feedback(-486000,-426000);
    UpStairMode().wall_alignment_r(0.03,0.04);
    r.sleep();
    UpStairMode().wall_following_r(0,0.05,0.1,0.06,0.8,0.75,1);
    r.sleep();
    UpStairMode().robotspeed(350,350);
    r.sleep();
    //===Turn left 90 degree===
    UpStairMode().robotmotion("Left");
    r.sleep();
    for (int i=1;i<10;i++)//i<10
    {
        sleep(1);
        std::cout<<"sleep for "<<i<<"sec\n";
    }
    UpStairMode().robotmotion("Stop");
    r.sleep();
    //===Wall following (right)===
    UpStairMode().wall_alignment_r(0.03,0.04);
    r.sleep();
    UpStairMode().wall_following_r(0,0.05,0.1,0.06,0.65,0.6,0.8);
    r.sleep();
    //===Turn left 90 degree===
    UpStairMode().robotspeed(350,350);
    r.sleep();
    UpStairMode().robotmotion("Left");
    r.sleep();
    for (int i=1;i<10;i++)
    {
        sleep(1);
        std::cout<<"sleep for "<<i<<"sec\n";
    }
    UpStairMode().robotmotion("Stop");

    UpStairMode().wall_alignment_r(0.03,0.04);
    r.sleep();
    //===Second Upstair===
    UpStairMode().mode_23(0.1,0.5,0.45,0.8,false);
    UpStairMode().mode_3();
    UpStairMode().mode_41("Right",20);
    //    UpStairMode().mode_4();
    //caliberate the motor by servo off
    DownStairMode().motor_switch(1,0);
    usleep(500000);
    DownStairMode().motor_switch(4,0);
    usleep(500000);
    sleep(1);
    DownStairMode().motor_switch(1,1);
    usleep(500000);
    DownStairMode().motor_switch(4,1);
    usleep(500000);
    DownStairMode().leghome(0,0);
    sleep(1);
    DownStairMode().dynamixel_motor(0,0);
    sleep(1);
}
void DemoMode::UpstairandDownstair1()
{
    ros::Rate r(2);
    printf("Upstair & Downstair Start!\n");
    //Initial motors
    DownStairMode().leghome(0,0);
    sleep(1);
    DownStairMode().dynamixel_motor(0,0);
    sleep(1);
    UpStairMode().legspeed(2500,2500);
    sleep(1);
    UpStairMode().legMA(-436000,-426000);
    sleep(6);
    //Down Stair
    UpStairMode().mode_23(0.1,0.5,0.45,0.8,false);
    UpStairMode().mode_3();
    UpStairMode().mode_41("Right",10);
    //    UpStairMode().mode_4();

    //caliberate the motor by servo off
    DownStairMode().motor_switch(1,0);
    usleep(500000);
    DownStairMode().motor_switch(4,0);
    usleep(500000);
    sleep(1);
    DownStairMode().motor_switch(1,1);
    usleep(500000);
    DownStairMode().motor_switch(4,1);
    usleep(500000);
    //lift front &back leg then turn around
    UpStairMode().legspeed(2000,2000);
    sleep(1);
    UpStairMode().legMA_feedback(-486000,-426000);
    UpStairMode().wall_alignment_r(0.03,0.04);
    r.sleep();
    UpStairMode().robotspeed(700,700);
    r.sleep();
    UpStairMode().robotmotion("Forward");
    sleep(5);
    UpStairMode().robotmotion("Left");
    r.sleep();
    for (int i=1;i<10;i++)
    {
        sleep(1);
        std::cout<<"sleep for "<<i<<"sec\n";
    }
    UpStairMode().robotmotion("Stop");
    r.sleep();
    UpStairMode().leghome(-486000,-426000);
    r.sleep();
    UpStairMode().wall_alignment_l(0.02,0.04);
    r.sleep();
//        //2nd Down Stair
    DownStairMode().mode_51(0,0.05,0.08,0.04,20);
    DownStairMode().mode_6();
//    DownStairMode().mode_7();
    DownStairMode().mode_71("Left");
    //caliberate the motor by servo off
    DownStairMode().motor_switch(1,0);
    usleep(500000);
    DownStairMode().motor_switch(4,0);
    usleep(500000);
    sleep(1);
    DownStairMode().motor_switch(1,1);
    usleep(500000);
    DownStairMode().motor_switch(4,1);
    usleep(500000);
}

void DemoMode::Floor4_Floor5()
{
        ros::Rate r(2);




        /*************************
         *** 4F Navigation **
         *************************/
        DownStairMode().dynamixel_motor(0,0);
        sleep(1);
        UpStairMode().legspeed(2500,2500);
        sleep(1);
        UpStairMode().legMA(-436000,-426000);
        sleep(6);
        robot().change_map(4);
        sleep(3);
        robot().Posepub(21.593,-16.039,0,0,0,1,-0.012,-3.117,1);//cirlab pose

        vector <double> len ={18,16.5,2.8};
        vector <double> ang ={0,-90,0};

        robot().savetotxt("/home/nvidia/move_data/");
        robot().pathplanning(len,ang,"4F_lab",0.2,21.593,-16.039,180);
        robot().Custom_path("xr_4F_lab","yr_4F_lab",1);
        robot().stop();

        ConsecutiveUpstair1("4");
        /*************************
         *** 5F Navigation **
         *************************/

        robot().change_map(5);
        sleep(1);
        UpStairMode().legspeed(2000,2000);
        sleep(1);
        UpStairMode().legMA(-20000,0);
        sleep(7);
        robot().Custom_path("xr_5f","yr_5f",0);
//        robot.NTNU_5Fpath();
        robot().stop();




}


