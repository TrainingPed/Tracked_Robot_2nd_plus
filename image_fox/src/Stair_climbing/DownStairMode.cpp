#include "ros/ros.h"
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64.h"

#include "DownStairMode.h"

#define  _UPSTAIRMODE_CPP_


//  extern float top,bottom;
//  extern float topp,bottomm;
//  extern float angg; //34
//  extern float ang;
static float top=2.5,bottom=0;
static float topp=0,bottomm=0;
static float angg=30; //34
static float ang=0;

void DownStairMode::check_us()
{
    ros::Rate r(1);
    while(ros::ok()&&!DownStairMode::safe_switch)
    {
        ros::spinOnce();
        printf("\n");
        printf("Checking ultrasonic data ......\t\t us = %d\n",DownStairMode::us);

        //printf("d21=%d\n",USM.d21);
        if(DownStairMode::us!=0)
        {
            printf("Connect to ultrasonic successful ! \n\n");
            break;
        }
        printf("Fail ! Retry Once ! \n");
        r.sleep();
    }
}

bool DownStairMode::leg_angle(int number, int angle)
{
    ros::spinOnce();
    int th1=100;
    if(abs(Encoders[number-1]-angle)<th1)
        return true;
    else
        return false;
}

void DownStairMode::ReadEnc2(const std_msgs::Int32MultiArray::ConstPtr &array)
{
    for (int j = 0; j < 4; j++)
    {
        Encoders[j] = *(array->data.begin() + j);
    }
    //    std::cout << "Encoder 1: " << Encoders[0]  << " Encoder 2: " <<Encoders[1]  << std::endl;
    //    std::cout << "Encoder 3: " << Encoders[2]  << " Encoder 4: " <<Encoders[3] << std::endl;

}

void DownStairMode::legMA_feedback(int front, int back)
{
    ros::Rate u(1);
    while(ros::ok())
    {

        std_msgs::Int32MultiArray leg_MA;

        l_MA[0]=front;
        l_MA[1]=back;
        for (int i = 0; i < 2; i++)
        {
            leg_MA.data.push_back(l_MA[i]);
        }
        pub_lMA.publish(leg_MA);
        printf("looping \n");
//        std::cout<<"front:"<<UpStairMode::leg_angle(1,front)<<std::endl;

//        std::cout<<"back:"<<UpStairMode::leg_angle(4,back)<<std::endl;
        ros::spinOnce();
        u.sleep();
        if(DownStairMode::leg_angle(1,front)&&(DownStairMode::leg_angle(4,back)))
            break;
    }
}

void DownStairMode::ultrasonicCallback(const std_msgs::Int32::ConstPtr& msg_us)
{
    us=msg_us->data;
    // printf("us=%d",us);
}
void DownStairMode::d5Callback(const std_msgs::Int32::ConstPtr& msg_d5)
{
    d5=msg_d5->data;
}
void DownStairMode::d21Callback(const std_msgs::Int32::ConstPtr& msg_d)
{
    d21=msg_d->data;
}
void DownStairMode::d7Callback(const std_msgs::Int32::ConstPtr& msg_d7)
{
    d7=msg_d7->data;
}

void DownStairMode::MD21callback(const std_msgs::Int32::ConstPtr &msg)
{
    MD21 = msg->data;
}
// void DownStairMode::midCallback(const std_msgs::Int32MultiArray::ConstPtr& array_mid)
// {
// 	mid_x=*(array_mid->data.begin());
// 	mid_y=*(array_mid->data.begin())+1;
// 	//ROS_INFO("mid:(%d,%d)",mid_x,mid_y);
// }
void DownStairMode::mid5Callback(const std_msgs::Int32MultiArray::ConstPtr& array_mid)
{
    mid_x5=*(array_mid->data.begin());
    mid_y5=*(array_mid->data.begin())+1;
    //ROS_INFO("mid:(%d,%d)",mid_x,mid_y);
}
void DownStairMode::mid7Callback(const std_msgs::Int32MultiArray::ConstPtr& array_mid)
{
    mid_x7=*(array_mid->data.begin());
    mid_y7=*(array_mid->data.begin())+1;
    //ROS_INFO("mid:(%d,%d)",mid_x,mid_y);
}
void DownStairMode::ustop_callback(const std_msgs::Int32::ConstPtr &msg){
    safe_switch=msg->data;
}

void DownStairMode::depth_scan_callback(const std_msgs::Float32MultiArray msg_depth)
{
    align_signal = 0;
    signal[0] = (int)msg_depth.data[0];
    signal[1] = (int)msg_depth.data[1];
    align_signal = (double)msg_depth.data[2];
//    std::cout<<(int)msg_depth.data[0]<<std::endl;
}

bool DownStairMode::depth_scan_hit_downstair(int limit_0,int limit_1)
{
    if(signal[0] > limit_0)
    {
        std::cout<<"Nan count : "<<signal[0]<<" Not Yet "<<std::endl;
        return false;
    }
    else
    {
        std::cout<<"Nan count : "<<signal[0]<<" <<-----------Signal[0] Hit !!"<<std::endl;
//        if(signal[1] > limit_1)
//        {
//            std::cout<<"Nan count : "<<signal[1]<<" Not Yet "<<std::endl;
//            return false;
//        }
//        else
//        {
//            std::cout<<"Nan count : "<<signal[1]<<" <<-----------Signal[1] Hit !!"<<std::endl;
//            return true;
//        }
        return true;
    }
}

void DownStairMode::mode_5()	//---Before 'down' stairs alignment
{

    std_msgs::Int32MultiArray robot_speed,leg_speed,robot_MA,leg_MA,robot_HO,robot_VA,robot_MR,leg_HO,leg_VA;

    std_msgs::Int32 robot_motion,leg_motion;

    dynamixel_workbench_msgs::JointCommand joint_command;

    // std_msgs::Float64 head_top,head_bottom;

    robot_speed.data.clear();
    leg_speed.data.clear();
    robot_MA.data.clear();
    leg_MA.data.clear();
    robot_HO.data.clear();
    robot_VA.data.clear();
    robot_MR.data.clear();

    /*---For Down Stairs Alignment(Mode 5) Luck---*/

    /*---Out While---*/
    int d=1000;

    /*---Vel - For Record---*/
    int vel_L=0,vel_R=0;

    //XtionMotor(550,0); //Put in mode 9
    DownStairMode::dynamixel_motor(0,10);
    sleep(1);
    printf("Mode 5 , Before down stairs Alignment \n");


    DownStairMode::robotspeed(350,350);

    usleep(50000);



    DownStairMode::robotmotion("Forward");


    usleep(500000);

    while(!safe_switch&&ros::ok())
    {


        DownStairMode::robotmotion("Forward");
        usleep(50000);



        if(mid_x5!=0 && mid_y5!=0)
        {

            if(abs(d5)<=22) //25
            {
                printf("mode5 , Alignment OK!\n");

                DownStairMode::robotmotion("Forward");
                usleep(300000);
                break;
            }
            else if(d5<=-23) //26
            {
                printf("d5=%d , Need Left\n",d5);
                DownStairMode::robotmotion("Stop");
                usleep(50000);
                DownStairMode::robotspeed(350,350);
                usleep(50000);
                DownStairMode::robotmotion("Left");
                usleep(300000);

            }
            else if(d5>=23) //26
            {
                printf("d5=%d , Need Right\n",d5);
                DownStairMode::robotmotion("Stop");

                usleep(50000);

                DownStairMode::robotspeed(350,350);
                usleep(50000);
                DownStairMode::robotmotion("Right");

                usleep(300000);

            }


        }//if end

        else //can't find stairs
        {
            printf("Can't Find Stairs! 'Patorl Mode'\n");
            DownStairMode::robotmotion("Stop");

            usleep(300000);

            d=1000;
        }

    }

}//mode5 end

void DownStairMode::mode_51(double lw_min, double lw_max, double vv, double ww, int md21)
{


    double sum_d23=0;
    int err=15;
    float Kp=1.0,Ki=0.1;
    ros::param::get(NameSpace+"/Kp", Kp);
    ros::param::get(NameSpace+"/Ki", Ki);
    ros::param::get(NameSpace+"/err", err);

    //    ros::param::get(NameSpace+"/vv", vv);

    //    std::cout<<"err="<<err<<"\n";
    //    std::cout<<"stop_align_dis="<<stop_align_dis<<"\n";

    /*---Vel - For Record---*/
    int vel_L=0,vel_R=0;
    int cnt=0;


    printf("Left Wall Following start ...\n");
    //=======ROS============
    std_msgs::Int32MultiArray robot_speed,leg_speed,robot_MA,leg_MA,robot_HO,robot_VA,robot_MR,leg_HO,leg_VA;

    std_msgs::Int32 robot_motion,leg_motion;

    dynamixel_workbench_msgs::JointCommand joint_command;
    robot_speed.data.clear();
    leg_speed.data.clear();
    robot_MA.data.clear();
    leg_MA.data.clear();
    robot_HO.data.clear();
    robot_VA.data.clear();
    robot_MR.data.clear();

    ros::Rate r(2);
    DownStairMode::dynamixel_motor(0,-20);
    sleep(1);
    DownStairMode::robotspeed(350,350);
    r.sleep();
    DownStairMode::robotmotion("Forward");
    r.sleep();
    int condition=0;
    while(ros::ok()&&!safe_switch)
    {

        ros::spinOnce();
        double Las_err=laser_481-laser_511;
        ros::Rate us_sleep(10);
        std::cout << "  us = "<<us<< std::endl;

        if( us>20&&us<100)//MD21>=md21
        {
           DownStairMode::robotmotion("Stop");
           sleep(1);
           printf("cnt=%d\n",cnt);
           break;
        }
        else if(Las_err<lw_min )
        {
            if(condition==1)
                continue;
            DownStairMode::vw_control(vv,-ww);//turn right
            printf("vv=%f,ww=%f\n",vv,-ww);
            condition=1;
            us_sleep.sleep();

        }
        else if(Las_err>lw_max)
        {
            if(condition==2)
                continue;
            DownStairMode::vw_control(vv,ww);//turn left
            printf("vv=%f,ww=%f\n",vv,ww);
            condition=2;
            us_sleep.sleep();
        }
        else
        {
            if(condition==3)
                continue;
            DownStairMode::vw_control(vv,0);//go straight
            printf("vv=%f,ww=0\n",vv);
            condition=3;
            us_sleep.sleep();
        }


    }

//    DownStairMode::robotmotion("Stop");
//    sleep(1);
    DownStairMode::legMA(0,0);
    sleep(8);
    //caliberate the motor by servo off
//    DownStairMode::motor_switch(1,0);
//    r.sleep();
//     DownStairMode::motor_switch(4,0);
//    r.sleep();
//    sleep(1);
//    DownStairMode::motor_switch(1,1);
//    r.sleep();
//    DownStairMode::DownStairMode::motor_switch(4,1);
//    r.sleep();

    std::cout<<"Stop Alignment\n";


}

void DownStairMode::mode_6()
{
    std_msgs::Int32MultiArray robot_speed,leg_speed,robot_MA,leg_MA,robot_HO,robot_VA,robot_MR,leg_HO,leg_VA;

    std_msgs::Int32 robot_motion,leg_motion;

    dynamixel_workbench_msgs::JointCommand joint_command;
    ros::param::set("current_limit",10000);//expand the current limit
    int count=0;
    robot_speed.data.clear();
    leg_speed.data.clear();
    robot_MA.data.clear();
    leg_MA.data.clear();
    robot_HO.data.clear();
    robot_VA.data.clear();
    robot_MR.data.clear();

    ros::Rate r(2);

    DownStairMode::legspeed(200,200);
    usleep(300000);
    DownStairMode::legMA(0,0);

    joint_command.request.unit = "rad";
    joint_command.request.id = 2;
    joint_command.request.goal_position = bottomm*PI/180;
    joint_command_client.call(joint_command);

    for(int i = 0 ; i <= 20 ; i++)
    {
        topp = 2*i;
        joint_command.request.unit = "rad";
        joint_command.request.id = 1;
        joint_command.request.goal_position = topp*PI/180;
        joint_command_client.call(joint_command);
        usleep(1000);
    }

    DownStairMode::legMA_feedback(0,0);
    usleep(300000);
    r.sleep();

    printf("Mode 6 , Down stairs prepare \n");

    DownStairMode::robotspeed(350,350);

    r.sleep();

    //Align before downstair
//    printf("Start to align before downstair \n");
//    while(ros::ok()&&!safe_switch)
//    {
//        if(!((std::isinf(laser_135))||(std::isinf(laser_90)))) //if the robot isn't going straight
//        {
//            align_laser=laser_135-laser_90;
//            if( align_laser>=0.4 )//  need to turn right
//            {
//                DownStairMode::vw_control(0,-0.01);
//                usleep(300000);
//                printf("Turn Right\n");
//            }
//            else if( align_laser<=0.2)// need to turn left
//            {
//                DownStairMode::vw_control(0,0.01);
//                usleep(300000);
//                 printf("Turn left\n");
//            }
//            else if (align_laser>=0.31 || align_laser<=0.2)
//            {
//                printf("Staight Ready to Go\n");
//                break;
//                }
//            }
//        }

    /*---Check Sensor Connection---*/


    if(us<1)
    {
        printf("*** Sensor Disconnection ***\n\n");


        DownStairMode::robotmotion("Stop");

        r.sleep();

    }
    else if(us>30)
    {
        DownStairMode::robotmotion("Stop");

        r.sleep();

    }
    else
    {
        printf("Ready to Close Stairs......\n");
        DownStairMode::robotmotion("Forward");
        usleep(100000);
        /*---If sensor have connection, Do---*/

        while(ros::ok()&&!safe_switch)
        {
            printf("HC sensor data = %d\n",us);

            ros::spinOnce();
            usleep(100000);
            if(us>25)
            {
                DownStairMode::robotmotion("Stop");
                usleep(300000);
                break;

            }
            else
            {
                DownStairMode::robotmotion("Forward");
                usleep(300000);
            }


        }
    }


    sleep(1);

    /*---When close stairs---*/
    printf("Down Stairs Prepare.\n\n");
    //---Down Prepare
    //bodyJOGvel(350,350);
    DownStairMode::robotspeed(350,350);
    usleep(300000);

    DownStairMode::legspeed(250,100);
    usleep(300000);
    DownStairMode::robotmotion("Stop");
    usleep(300000);
    //legcs(0,0);
    DownStairMode::legMA_feedback(0,0);
    sleep(1);
    //	usleep(30000);
    //	usleep(100000);
    //legma(100*300,(90+angg)*300);
    DownStairMode::robotmotion("Forward");
    usleep(300000);
    DownStairMode::legMA_feedback(-9500,-8000);//89000 ,-73000
    usleep(100000);

    //bodyact("fwd");
    DownStairMode::legspeed(450,50);
    usleep(100000);
    DownStairMode::legMA_feedback(300,-3000);

    usleep(300000);
    //	usleep(5800000);

    //legma(98*300,95*300);
    //    DownStairMode::legMA(98*300,95*300);

    //	usleep(7000000); //5.5sec

    //bodyact("stop");
    DownStairMode::robotmotion("Stop");
    usleep(500000);

    printf("END Down Stairs Prepare .\n\n");
}

void DownStairMode::mode_7()	//---'down' stairs alignment
{

    std_msgs::Int32MultiArray robot_speed,leg_speed,robot_MA,leg_MA,robot_HO,robot_VA,robot_MR,leg_HO,leg_VA;

    std_msgs::Int32 robot_motion,leg_motion;

    dynamixel_workbench_msgs::JointCommand joint_command;

    robot_speed.data.clear();
    leg_speed.data.clear();
    robot_MA.data.clear();
    leg_MA.data.clear();
    robot_HO.data.clear();
    robot_VA.data.clear();
    robot_MR.data.clear();

    ros::Rate r(1);

    int d7d=1000;


    /*---Data Record - Open---*/
    //    FILE *M7XD=fopen("/home/ubuntu/CodeBlocks/TrackedRobot_Control/Recore Data/M7_XtionDistance.txt","w");
    //    FILE *M7PL=fopen("/home/ubuntu/CodeBlocks/TrackedRobot_Control/Recore Data/M7_PixelLocation.txt","w");
    //    FILE *M7VL=fopen("/home/ubuntu/CodeBlocks/TrackedRobot_Control/Recore Data/M7_VelLeft.txt","w");
    //    FILE *M7VR=fopen("/home/ubuntu/CodeBlocks/TrackedRobot_Control/Recore Data/M7_VelRight.txt","w");
    //    FILE *M7clcr=fopen("/home/ubuntu/CodeBlocks/TrackedRobot_Control/Recore Data/M7_clcr.txt","w");
    //    FILE *M7DD=fopen("/home/ubuntu/CodeBlocks/TrackedRobot_Control/Recore Data/M7_downDistance.txt","w");
    /*---Vel - For Record---*/

    int vel_L=0,vel_R=0;
    int count=0;
    int counter=0;
    //XtionMotor(600,0);

    bottomm=bottom;
    topp=top+40;
    joint_command.request.unit = "rad";
    joint_command.request.id = 2;
    joint_command.request.goal_position = bottomm*PI/180;
    joint_command_client.call(joint_command);
    joint_command.request.unit = "rad";
    joint_command.request.id = 1;
    joint_command.request.goal_position = topp*PI/180;
    joint_command_client.call(joint_command);
    ROS_INFO("bottom:%f,top:%f", bottomm,topp);
    sleep(1);

    printf("Mode 7 , Down stairs alignment \n");

    //bodyact("fwd");
    //    DownStairMode::robotmotion("Forward");


    DownStairMode::robotspeed(1000,1000);
    usleep(300000);
    DownStairMode::robotmotion("Forward");
    usleep(300000);
    int flag = 0;

    while(ros::ok()&&!safe_switch)
    {
        ros::spinOnce();
        //------ Stop if it reaches the bottom ------//
        if( (DownStairMode::depth_scan_hit_downstair(2,100)))
        {
            DownStairMode::robotmotion("Stop");
            printf("Ready to lift arm!!");
            usleep(300000);
            break;
        }
        else
        {
            printf("Climbling Align");
            if( laser_135 < 0.95 )
            {
                flag = 1;
                printf("\t--->\t Data out of range !! Go Straight\n");
                DownStairMode::vw_control( 0.1, 0);
                usleep(300000);
            }
//            else if (counter<5)
//            {
//                printf("Stair distance Align!!/n");
//                if(laser_90<0.3) //too close to wall turn right
//                {
//                    printf("Too close! Turn right!/n");
//                    DownStairMode::vw_control(0.05,-0.04);
//                    sleep(1);
//                    DownStairMode::vw_control(0.05,0.04);
//                    sleep(1);
//                    printf("counter=%d",counter);
//                    counter++;
//                }
//                else if(laser_90>0.3)  // too far from the wall turn left
//                {
//                    printf("Too far! Turn left!/n");
//                    DownStairMode::vw_control(0.05,0.04);
//                    sleep(1);
//                    DownStairMode::vw_control(0.05,-0.04);
//                    sleep(1);
//                    printf("counter=%d",counter);
//                    counter++;
//                }
//                else
//                {
//                    printf("Go straignt!/n");
//                    DownStairMode::vw_control(0.05,0);
//                    printf("counter=%d",counter);
//                    counter ++;
//                    sleep(1);
//                }
//            }
            else if(!((std::isinf(laser_135))||(std::isinf(laser_90))||(flag == 1))) //if the robot isn't going straight and laser ins't nan
            {
                align_laser=laser_135-laser_90;
                if( align_laser>=1.35 )//  need to turn left
                {
                    printf("\t--->\t Turn left\n");
                    DownStairMode::vw_control(0.1,0.04);
                    usleep(300000);
                }
                else if( align_laser<=1.13)// need to turn right
                {
                    printf("\t--->\t Turn Right\n");
                    DownStairMode::vw_control(0.1,-0.04);
                    usleep(300000);
                }
                else
                {
                    printf("\t--->\t Go Straight\n");
                    DownStairMode::vw_control( 0.1, 0);
                    usleep(300000);
                }
            }
            else
            {
                printf("\t--->\t Invalid Data !! Go Straight\n");
                DownStairMode::vw_control( 0.1, 0);
                usleep(300000);
            }
        }


        //------Align the Route if it is still climbing ------//
//        else
//        {

////            int test_align = UpStairMode::depth_scan_align(-0.02);
//            UpStairMode::vw_control( 0.05, -0.01*(UpStairMode::depth_scan_align( -0.02 )) );//0.02*(UpStairMode::depth_scan_align( -0.02 ))
////            std::cout<<""<<std::endl;
//            usleep(300000);

//        }
    }

//    while(ros::ok()&&!safe_switch)
//        //while(d7d>800)//700
//    {

//        ros::spinOnce();
//        printf("laser260=%f\n",laser_260);
//        sleep(1);
//        //        if(mid_x7!=0&&mid_y7!=0)
//        //        {
//        if(laser_260 > 1 && laser_170 >1 && laser_340 >1)
//        {
//            if(abs(d21)<=30)
//            {

//                //printf("error:%d , Alignment OK!\n",d7);

//                DownStairMode::robotspeed(350,350);
//                usleep(500000);
//                DownStairMode::robotmotion("Forward");
//                usleep(500000);

//            }
//            else if(d21>30)
//            {

//                //printf("error:%d , need some left\n",d21);
//                DownStairMode::robotspeed(1000,1350);
//                usleep(500000);
//                DownStairMode::robotmotion("Forward");
//                usleep(500000);

//            }
//            else if(d21<-30)
//            {
//                //                printf("error:%d , need some right\n",d7);
//                DownStairMode::robotspeed(1350,1000);
//                usleep(500000);
//                DownStairMode::robotmotion("Forward");
//                usleep(500000);
//            }
//            count=0;
//        }
//        else
//        {
//            DownStairMode::robotmotion("Stop");
//            usleep(500000);
//            count++;
//        }
//        if(count>=3)
//        {
//            break;
//        }

//        //        }
//        //        else
//        //        {
//        //				printf("mode_7 : Stairs End\n");
//        //                DownStairMode::robotmotion("Stop");
//        //                usleep(50000);
//        //        }



//    }

    //----down stairs end , prepare-----
    printf("Landing prepare...\n");

//    DownStairMode::dynamixel_motor(0,0);
    usleep(1);
    //bodyact("stop");
    DownStairMode::robotmotion("Stop");
    usleep(300000);
    DownStairMode::robotspeed(700,700);
    usleep(300000);
    DownStairMode::robotmotion("Forward");
    DownStairMode::legspeed(80,200);
    usleep(300000);
//    DownStairMode::legMA()
    DownStairMode::legMA_feedback(10000,-3000);
    DownStairMode::legspeed(100,120);
    usleep(300000);
    DownStairMode::legMA_feedback(10000,16000);
    DownStairMode::legspeed(300,230);
    usleep(300000);
    DownStairMode::legMA(38000,29000);
    sleep(4);
    DownStairMode::robotmotion("Stop");
    for(int i = 25 ; i <= 0 ; i--)
    {
        topp = 2*i;
        joint_command.request.unit = "rad";
        joint_command.request.id = 1;
        joint_command.request.goal_position = topp*PI/180;
        joint_command_client.call(joint_command);
        usleep(10);
    }
    DownStairMode::legMA_feedback(38000,29000);
    usleep(300000);
    DownStairMode::robotmotion("Stop");
    usleep(300000);
    DownStairMode::legspeed(200,200);
    DownStairMode::legMA_feedback(0,0);
    usleep(300000);
    for( int i = 1; i <=4 ; i++ )
    {
        DownStairMode::motor_switch(i,0);
        usleep(300000);
    }
    sleep(2);
    for( int i = 1; i <=4 ; i++ )
    {
        DownStairMode::motor_switch(i,1);
        usleep(300000);
    }
    sleep(2);
    DownStairMode::robotspeed(350,350);
    usleep(300000);
    DownStairMode::robotmotion("Stop");
    usleep(300000);
    ros::param::set("current_limit",10000);//reset the current limit
    printf("Down stairs END ! \n");



}//mode7 end
void DownStairMode::mode_71(std::string direction)	//---'down' stairs alignment
{


    dynamixel_workbench_msgs::JointCommand joint_command;

    int vel_L=0,vel_R=0,vel_c=1350;
    double Las_err=0,Las_0=0,Las_30=0;


    ros::Rate r(1);

   /*---Vel - For Record---*/

    int count=0,condition=0;

    bottomm=bottom;
    topp=top+10;
    joint_command.request.unit = "rad";
    joint_command.request.id = 1;
    joint_command.request.goal_position = bottomm*PI/180;
    joint_command_client.call(joint_command);
    joint_command.request.unit = "rad";
    joint_command.request.id = 2;
    joint_command.request.goal_position = topp*PI/180;
    joint_command_client.call(joint_command);
    ROS_INFO("bottom:%f,top:%f", bottomm,topp);
    sleep(1);

    printf("Mode 7 , Down stairs alignment \n");


    DownStairMode::robotspeed(1350,1350);
    usleep(500000);

    DownStairMode::robotmotion("Forward");
    usleep(500000);
    while(ros::ok()&&!safe_switch)
    {
        std::cout<<"Las_err="<<Las_err<<"\n";
        std::cout<<"Las_0="<<Las_0<<"\n";
        std::cout<<"Las_30="<<Las_30<<"\n";
        if(direction=="Left"){
            Las_err=laser_481-laser_511;
            Las_0=laser_511;
            Las_30=laser_481;
            vel_L=350;
        }
        else if(direction=="Right")
        {
            Las_err=laser_32-laser_2;
            Las_0=laser_2;
            Las_30=laser_32;
            vel_R=350;
        }
        else
        {
            printf("Neither left nor right??\n");
        }

        ros::spinOnce();
        printf("laser260=%f\n",laser_260);
        sleep(1);

        if(laser_260 > 1 && laser_170 >1 && laser_340 >1)
        {
            if(laser_260<2)
            {
                vel_c = 700;
            }
            if(Las_err<0) //approach wall
            {
                if(laser_260 <2)
                {
                    if(condition==2)
                    condition=2;
                    DownStairMode::robotspeed(vel_c+vel_L,vel_c+vel_R);//adjust the pose first
                    usleep(500000);
                }
                else if(Las_0>1 && Las_30>1){//go straight
                    if(condition==1)
                    condition=1;
                    DownStairMode::robotspeed(vel_c,vel_c);
                    usleep(500000);
                }
                else{//go away
                    if(condition==2)
                    condition=2;
                    DownStairMode::robotspeed(vel_c+vel_L,vel_c+vel_R);
                    usleep(500000);
                }
                DownStairMode::robotmotion("Forward");
                usleep(500000);
                count=0;
            }
            else if(Las_err>0.02)//keep away from wall
            {
                if(laser_260 <2)
                {
                    if(condition==3)
                    condition=3;
                    DownStairMode::robotspeed(vel_c-vel_L,vel_c-vel_R);
                    usleep(500000);
                }
                else if(Las_0<0.8&&Las_0<0.8){//go straight
                    if(condition==1)
                    condition=1;
                    DownStairMode::robotspeed(vel_c,vel_c);
                    usleep(500000);
                }
                else    //appraoch the wall
                {
                    if(condition==3)
                    condition=3;
                    DownStairMode::robotspeed(vel_c-vel_L,vel_c-vel_R);
                    usleep(500000);
                }

                DownStairMode::robotmotion("Forward");
                usleep(500000);
                count=0;

            }
            else
            {
                if(laser_260<2)
                {
                    if(condition==1)
                    condition=1;
                    DownStairMode::robotspeed(vel_c,vel_c);//go straight
                    usleep(500000);
                }
                else if(Las_0>1)
                {
                    if(condition==3)
                    condition=3;
                    DownStairMode::robotspeed(vel_c-vel_L,vel_c-vel_R);//approach
                    usleep(500000);
                }
                else if(Las_0<0.8)
                {
                    if(condition==2)
                    condition=2;
                    DownStairMode::robotspeed(vel_c+vel_L,vel_c+vel_R);//go away
                    usleep(500000);
                }
                else
                {
                    if(condition==1)
                    condition=1;
                    DownStairMode::robotspeed(vel_c,vel_c);//go straight
                    usleep(500000);
                }
                DownStairMode::robotmotion("Forward");
                usleep(500000);
                count=0;
            }

        }
        else
        {
            DownStairMode::robotmotion("Stop");
            usleep(500000);
            count++;
            if(count>=2)
                break;

        }



    }

    //----down stairs end , prepare-----
    printf("Landing prepare...\n");

//    while(ros::ok()&&!safe_switch)
//    {

//        double Las_err=laser[350]-laser[150];
//        std::cout << "  laser_481 = "<<laser[350]<< std::endl;
//        std::cout << "  laser_511 = "<<laser[150]<< std::endl;
//        std::cout << "  Las_err = "<<Las_err<< std::endl;
//        if(Las_err<-0.1)
//        {
//            DownStairMode::vw_control(0,-0.02);//turn right
//            r.sleep();
////            printf("vv=%f,ww=%f\n",vv,-ww);

//        }
//        else if(Las_err>0.1)
//        {
//            DownStairMode::vw_control(0,0.02);//turn left
//            r.sleep();
////            printf("vv=%f,ww=%f\n",vv,-ww);

//        }
//        else if(Las_err<0)
//        {
//            DownStairMode::vw_control(0,-0.02);//turn right
//            r.sleep();
////            printf("vv=%f,ww=%f\n",vv,-ww);

//        }
//        else
//        {
//            DownStairMode::robotmotion("Stop");;//Stop
//            r.sleep();
//            break;
//        }
//    }
//    std::cout<<"Stop Alignment\n";

//    DownStairMode::dynamixel_motor(0,0);
//    sleep(1);
//    //bodyact("stop");
//    DownStairMode::robotmotion("Stop");
//    usleep(100000);
//    if(direction =="Left")
//        DownStairMode::wall_alignment_l(0,0.03);
//    else if(direction =="Right")
//        DownStairMode::wall_alignment_r(0,0.03);
    usleep(500000);
    DownStairMode::robotspeed(1350,1350);
    usleep(500000);
    DownStairMode::robotmotion("Forward");
    sleep(2);
    DownStairMode::robotspeed(700,700);
    usleep(500000);
    DownStairMode::legMA(-110000,-110000);
    sleep(1);
    DownStairMode::robotmotion("Forward");
    sleep(7);
    DownStairMode::robotmotion("Stop");
    sleep(1);
    DownStairMode::legMA(0,0);
    sleep(3);





    ros::param::set("current_limit",10000);//reset the current limit
    printf("Down stairs END ! \n");



}//mode7 end

//Handy Function
void DownStairMode::vw_control(float v,float w)
{
//    ros::Rate r(2);
    double R=0.06828;
    double W=0.84112;

    if(w>0.3)
        w=0.3;
    if(w<-0.3)
        w=-0.3;
    float left = v - (W / 2)*w;//v left
    float right = v + (W / 2)*w;//v right
    int M2 = (int)(left *Ratio * 60 / (2 * PI*R));//Motor 2 rpm
    int M3 = (int)(right *Ratio * 60 / (2 * PI*R));//Motor 3 rpm
    DownStairMode::robotspeed(M2,M3);
    usleep(100000);
    DownStairMode::robotmotion("Forward");
    usleep(100000);
}
void DownStairMode::wall_alignment_r(double rw_min,double rw_max)	//---Alignment with laser & Calculated the tilt angle
{

    double sum_d23=0;
    int d=800;
    int err=15;
    //    double Lfar_dis=0.5,Lclose_dis=0.45,Lstop_align_dis=1;
    float Kp=1.0,Ki=0.1;
    ros::param::get(NameSpace+"/Kp", Kp);
    ros::param::get(NameSpace+"/Ki", Ki);
    ros::param::get(NameSpace+"/err", err);

    /*---Vel - For Record---*/
    int vel_L=0,vel_R=0;
    int cnt=0;


    printf("Right Wall Following start ...\n");
    //=======ROS============
    std_msgs::Int32MultiArray robot_speed,leg_speed,robot_MA,leg_MA,robot_HO,robot_VA,robot_MR,leg_HO,leg_VA;

    std_msgs::Int32 robot_motion,leg_motion;

    dynamixel_workbench_msgs::JointCommand joint_command;
    robot_speed.data.clear();
    leg_speed.data.clear();
    robot_MA.data.clear();
    leg_MA.data.clear();
    robot_HO.data.clear();
    robot_VA.data.clear();
    robot_MR.data.clear();
    double vv=0,ww=0.04;
    ros::Rate r(2);

    DownStairMode::robotspeed(0,0);
    r.sleep();
    DownStairMode::robotmotion("Forward");
    r.sleep();

    printf("Start Angle aligment!\n");
    while(ros::ok()&&!safe_switch)
    {

        double Las_err=laser_32-laser_2;
        std::cout << "  laser32 = "<<laser_32<< std::endl;
        std::cout << "  laser2 = "<<laser_2<< std::endl;
        std::cout << "  Las_err = "<<Las_err<< std::endl;
        if(Las_err<rw_min)
        {
            DownStairMode::vw_control(0,ww);//turn left
//            sleep(1);
            printf("vv=%f,ww=%f\n",vv,ww);

        }
        else if(Las_err>rw_max)
        {
            DownStairMode::vw_control(0,-ww);//turn right
//            sleep(1);
            printf("vv=%f,ww=%f\n",vv,-ww);

        }
        else if(Las_err<0)
        {
            DownStairMode::vw_control(0,ww);//turn left
//            sleep(1);
            printf("vv=%f,ww=%f\n",vv,ww);

        }
        else
        {
            DownStairMode::robotmotion("Stop");;//Stop
//            sleep(1);
//            r.sleep();
            break;
        }
        r.sleep();
    }
    std::cout<<"Stop Alignment\n";





//    r.sleep();


    DownStairMode::robotmotion("Stop");
    r.sleep();
}
void DownStairMode::wall_alignment_l(double rw_min,double rw_max)	//---Alignment with laser & Calculated the tilt angle
{

    double sum_d23=0;
    int d=800;
    int err=15;
    //    double Lfar_dis=0.5,Lclose_dis=0.45,Lstop_align_dis=1;
    float Kp=1.0,Ki=0.1;
    ros::param::get(NameSpace+"/Kp", Kp);
    ros::param::get(NameSpace+"/Ki", Ki);
    ros::param::get(NameSpace+"/err", err);

    /*---Vel - For Record---*/
    int vel_L=0,vel_R=0;
    int cnt=0;


    printf("Left Wall Following start ...\n");
    //=======ROS============
    std_msgs::Int32MultiArray robot_speed,leg_speed,robot_MA,leg_MA,robot_HO,robot_VA,robot_MR,leg_HO,leg_VA;

    std_msgs::Int32 robot_motion,leg_motion;

    dynamixel_workbench_msgs::JointCommand joint_command;
    robot_speed.data.clear();
    leg_speed.data.clear();
    robot_MA.data.clear();
    leg_MA.data.clear();
    robot_HO.data.clear();
    robot_VA.data.clear();
    robot_MR.data.clear();
    double vv=0,ww=0.04;
    ros::Rate r(2);

    DownStairMode::robotspeed(0,0);
    r.sleep();
    DownStairMode::robotmotion("Forward");
    r.sleep();

    printf("Start Angle aligment!\n");
    while(ros::ok()&&!safe_switch)
    {

        double Las_err=laser_481-laser_511;
        std::cout << "  laser_481 = "<<laser_481<< std::endl;
        std::cout << "  laser_511 = "<<laser_511<< std::endl;
        std::cout << "  Las_err = "<<Las_err<< std::endl;
        if(Las_err<rw_min)
        {
            DownStairMode::vw_control(0,-ww);//turn right
            r.sleep();
            printf("vv=%f,ww=%f\n",vv,-ww);

        }
        else if(Las_err>rw_max)
        {
            DownStairMode::vw_control(0,ww);//turn left
            r.sleep();
            printf("vv=%f,ww=%f\n",vv,-ww);

        }
        else if(Las_err<0)
        {
            DownStairMode::vw_control(0,-ww);//turn right
            r.sleep();
            printf("vv=%f,ww=%f\n",vv,-ww);

        }
        else
        {
            DownStairMode::robotmotion("Stop");;//Stop
            r.sleep();
            break;
        }
    }
    std::cout<<"Stop Alignment\n";





    r.sleep();


    DownStairMode::robotmotion("Stop");
    r.sleep();
}
void DownStairMode::lasercallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    /*    double min,max;
    min = msg->angle_min;
    max = msg->angle_max;
    std::cout<<"min angle="<<min/PI*180<<" max angle="<<max/PI*180<<std::endl*/;
    laser_170 = msg->ranges[220];
    laser_260 = msg->ranges[260];
    laser_340 = msg->ranges[300];
    laser_2 =msg->ranges[2];
    laser_32 =msg->ranges[32];
    laser_481=msg->ranges[481];
    laser_511=msg->ranges[511];
    laser_135=msg->ranges[386];
    laser_90=msg->ranges[512];
    for(int i=0;i<512;i++){
        laser[i]=msg->ranges[i];
    }
    //    std::cout << "  laser170 = "<<laser_170<< std::endl;
    //    std::cout << "  laser340 = "<<laser_340<< std::endl;
    //    std::cout << "  laser260 = "<<laser_260<< std::endl;

}

void DownStairMode::dynamixel_motor(int X,int Y)
{
    dynamixel_workbench_msgs::JointCommand joint_command;
    bottomm=bottom-X;
    topp=top-Y;
    joint_command.request.unit = "rad";
    joint_command.request.id = 1;
    joint_command.request.goal_position = bottomm*PI/180;
    joint_command_client.call(joint_command);
    joint_command.request.unit = "rad";
    joint_command.request.id = 2;
    joint_command.request.goal_position = topp*PI/180;
    joint_command_client.call(joint_command);

    printf("ZED turn down\n");
    ROS_INFO("bottom:%f,top:%f", bottomm,topp);
}
void DownStairMode::legMA(int front,int back)
{			
    std_msgs::Int32MultiArray leg_MA;

    l_MA[0]=front;
    l_MA[1]=back;
    for (int i = 0; i < 2; i++)
    {
        leg_MA.data.push_back(l_MA[i]);
    }
    pub_lMA.publish(leg_MA);
    printf("leg_MA publish\n");

    ros::spinOnce();
}
void DownStairMode::robotMA(int left,int right)
{
    std_msgs::Int32MultiArray robot_MA;

    r_MA[0]=left;
    r_MA[1]=right;
    for (int i = 0; i < 2; i++)
    {
        robot_MA.data.push_back(r_MA[i]);
    }
    pub_rMA.publish(robot_MA);
    printf("robot_MA publish\n");

    ros::spinOnce();
}
void DownStairMode::robotspeed(int left,int right)
{
    std_msgs::Int32MultiArray robot_speed;

    r_speed[0]=left;
    r_speed[1]=right;
    for (int i = 0; i < 2; i++)
    {
        robot_speed.data.push_back(r_speed[i]);
    }
    pub_rs.publish(robot_speed);
    std::cout<<"left ="<<left<<", right="<<right<<std::endl;
//    printf("robot_speed publish\n");

    ros::spinOnce();
}
void DownStairMode::legspeed(int back,int front)
{
    std_msgs::Int32MultiArray leg_speed;

    l_speed[0]=back;
    l_speed[1]=front;
    for (int i = 0; i < 2; i++)
    {
        leg_speed.data.push_back(l_speed[i]);
    }
    pub_ls.publish(leg_speed);
    printf("legspeed publish\n");
    std::cout<<"front ="<<front<<", back="<<back<<std::endl;

    ros::spinOnce();
}

void DownStairMode::robotmotion(std::string rm)
{
    std_msgs::Int32 robot_motion;


    if  (rm =="Stop")
        robot_motion.data=0;
    else if(rm =="Forward")
        robot_motion.data=1;
    else if(rm =="Backward")
        robot_motion.data=2;
    else if(rm =="Left")
        robot_motion.data=3;
    else if(rm =="Right")
        robot_motion.data=4;
    else
        std::cout<<"error rm input!!\n";

    pub_rm.publish(robot_motion);
    std::cout<<"robot_motion "<<rm<<" publish\n";

    ros::spinOnce();
}
void DownStairMode::legmotion(std::string lm)
{

    std_msgs::Int32 leg_motion;

    if  (lm=="Front_Up")
        leg_motion.data=1;
    else if(lm=="Front_Down")
        leg_motion.data=2;
    else if(lm=="Back_Up")
        leg_motion.data=3;
    else if(lm=="Back_Down")
        leg_motion.data=4;
    else
        std::cout<<"error lm input!!\n";


    pub_lm.publish(leg_motion);
    printf("leg_motion publish\n");

    ros::spinOnce();
}
void DownStairMode::leghome(int front,int back)
{
    std_msgs::Int32MultiArray leg_HO;

    l_HO[0]=back;
    l_HO[1]=front;
    for (int i = 0; i < 2; i++)
    {
        leg_HO.data.push_back(l_HO[i]);
    }
    pub_lHO.publish(leg_HO);
    printf("leghome publish\n");

    ros::spinOnce();
}
void DownStairMode::motor_switch(int number,int serve_status)
{
    std_msgs::Int32MultiArray motor_switch;

    MS[0]=number;
    MS[1]=serve_status;
    for (int i = 0; i < 2; i++)
    {
        motor_switch.data.push_back(MS[i]);
    }
    pub_MS.publish(motor_switch);
    printf("motor_switch publish\n");

    ros::spinOnce();
}

