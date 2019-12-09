#include "ros/ros.h"
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <string>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

// #include "ZED_init.h"
#include "UpStairMode.h"

#define  _UPSTAIRMODE_CPP_

// float top=-5,bottom=30;
// float topp=-5,bottomm=30;
// float angg=30; //34
// float ang=0;
static float top=0,bottom=2.5,front_d=0;
static float topp=0,bottomm=0,front_dd=0;
static float angg=30; //34
static float ang=0;

double UpStairMode::WallAlign( double mid , double left , double right )
{

    align_laser = laser_150 - laser_210;
    while(ros::ok())
    {
        printf("align_laser : %.3f" , align_laser );
        if( align_laser >= left)    //Need to turn Right
        {
            printf("Turn Right\n");
            vw_control(0,0.01);
        }
        else if( align_laser <= right) //Need to turn Left
        {
            printf("Turn Left\n");
            vw_control(0,0.01);
        }
        else {
            printf("Stop Alignment\n");
            vw_control(0,0);
            break;
        }
        ros::spinOnce();
    }
    while(ros::ok())
    {
        if( laser_180 >= mid )
        {
            UpStairMode::vw_control(0.01,0);
            printf("Too Far\n");
        }
        else if(laser_180 <= mid )
        {
            UpStairMode::vw_control(-0.01,0);
            printf("Too Close\n");
        }
        else
        {
            UpStairMode::vw_control(0,0);
            printf("Stop\n");
            break;
        }
        ros::spinOnce();
    }
    printf("End of WallAlign\n");
}

double UpStairMode::BubbleSort(int array_size, double array[])
{

    double t, temp[array_size-1];
    for (int ii = 0; ii < array_size; ii++)
        temp[ii] = array[ii];
    for (int i = 0; i < array_size - 1; i++)
    {
        for (int j = array_size - 1; j > i; j--)
        {
            if (temp[j] < temp[j - 1])
            {
                t = temp[j];
                temp[j] = temp[j - 1];
                temp[j - 1] = t;
            }
        }
    }

    return temp[array_size / 2];
}

void UpStairMode::ReadEnc2(const std_msgs::Int32MultiArray::ConstPtr &array)
{
    for (int j = 0; j < 4; j++)
    {
        Encoders[j] = *(array->data.begin() + j);
    }
    //    std::cout << "Encoder 1: " << Encoders[0]  << " Encoder 2: " <<Encoders[1]  << std::endl;
    //    std::cout << "Encoder 3: " << Encoders[2]  << " Encoder 4: " <<Encoders[3] << std::endl;

}
void UpStairMode::us_callback(const std_msgs::Int32::ConstPtr &msg)
{
    ultrasonic = msg->data;
}

void UpStairMode::d21Callback(const std_msgs::Int32::ConstPtr& msg_d21)
{	
    d21=msg_d21->data;
}
void UpStairMode::d4Callback(const std_msgs::Int32::ConstPtr& msg_d4)
{	
    d4=msg_d4->data;
}
void UpStairMode::MD21callback(const std_msgs::Int32::ConstPtr &msg)
{
    MD21=msg->data;
}
void UpStairMode::cnt21Callback(const std_msgs::Int32::ConstPtr& msg_cnt21)
{
    counter21=msg_cnt21->data;
}
void UpStairMode::midCallback(const std_msgs::Int32MultiArray::ConstPtr& array_mid)
{
    mid_x=*(array_mid->data.begin());
    mid_y=*(array_mid->data.begin())+1;
    //ROS_INFO("mid:(%d,%d)",mid_x,mid_y);
}
void UpStairMode::ustop_callback(const std_msgs::Int32::ConstPtr &msg){
    safe_switch=msg->data;
}

void UpStairMode::depth_scan_callback(const std_msgs::Float32MultiArray msg_depth)
{
    align_signal = 0;
    signal[0] = (int)msg_depth.data[0];
    signal[1] = (int)msg_depth.data[1];
    align_signal = (float)msg_depth.data[2];
    left_signal = (float)msg_depth.data[3];
    right_signal = (float)msg_depth.data[4];
}
int UpStairMode::depth_scan_align()
{

    if( -0.02 < align_signal && align_signal < 0.02)
    {
        printf("\nGo Straight !\n");
        return 0;
    }

    else if( left_signal == 0 || right_signal == 0)
    {
        printf("\nGo Straight !\n");
        return 0;
    }

    else if(align_signal > 0.02)
    {

        if( 70 <= signal[0] && signal[0] <= 80 && 30 <= signal[1] &&signal[1] <= 40 )
        {
            printf("\nTurn Right !\t\tAlign_Signal : %f\n" , align_signal);
            return -1;
        }
        else
        {
            printf("\nGo Straight !\n");
            return 0;
        }

    }

    else if(align_signal < -0.02)
    {

        if( 50 <= signal[0] && signal[0] <= 65 && 30 <= signal[1] &&signal[1] <= 40 )
        {
            printf("\nTurn Left !\t\tAlign_Signal : %f\n" , align_signal);
            return 1;
        }
        else
        {
            printf("\nGo Straight !\n");
            return 0;
        }
    }

//    else
//    {
//        printf("\nGo Straight !\n");
//        return 0;
//    }

}

bool UpStairMode::depth_scan_hit(int limit_0,int limit_1)
{
    if(signal[0] < limit_0)
    {
        std::cout<<"Nan count : "<<signal[0]<<" Not Yet "<<std::endl;
        return false;
    }
    else
    {
        std::cout<<"Nan count : "<<signal[0]<<" <<-----------Signal[0] Hit !!"<<std::endl;
        if(signal[1] < limit_1)
        {
            std::cout<<"Nan count : "<<signal[1]<<" Not Yet "<<std::endl;
            return false;
        }
        else
        {
            std::cout<<"Nan count : "<<signal[1]<<" <<-----------Signal[1] Hit !!"<<std::endl;
            return true;
        }
    }
}
void UpStairMode::vodom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    vox = msg->pose.pose.position.x;
    voy = msg->pose.pose.position.y;
    tf::Pose pose;
    tf::poseMsgToTF(msg->pose.pose, pose);
    tf::Matrix3x3(pose.getRotation()).getRPY(roll,pitch,yaw);
//    std::cout<<"pitch="<<pitch<<" \n";

}


void UpStairMode::start()
{
    ros::Rate r(2);
    while(ros::ok()&&!safe_switch)
    {
        ROS_INFO("d21:%d , cnt21:%d",d21,counter21);
        if(d21!=0&&counter21!=0)break;
        ros::spinOnce();
        r.sleep();
    }
}

bool UpStairMode::leg_angle(int number, int angle)
{
    ros::spinOnce();
    int th1=100;
    //printf("%d\n",Encoders[number-1]);
    if(abs(Encoders[number-1]-angle)<th1)
        return true;
    else
        return false;
}
void UpStairMode::legMA_feedback(int front, int back)
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
        //std::cout<<"legMA: "<<leg_MA.data[0]<<","<<leg_MA.data[1]<<std::endl;
        printf("looping \n");
//        std::cout<<"front:"<<UpStairMode::leg_angle(1,front)<<std::endl;

//        std::cout<<"back:"<<UpStairMode::leg_angle(4,back)<<std::endl;
        ros::spinOnce();
        u.sleep();
        if(UpStairMode::leg_angle(1,front)&&(UpStairMode::leg_angle(4,back)))
            break;
    }
}
void UpStairMode::mode_align()
{
    double align_laser;
    while(ros::ok())
    {
        ros::spinOnce();
        align_laser = Align180();
        printf("Align_laser : %f \n", align_laser);
        if( align_laser>=0.4 )//  need to turn left
        {
             UpStairMode::vw_control(0,0.02);
             usleep(300000);
             printf("Turn Left\n");
        }
        else if( align_laser<=0.35)// need to turn right
        {
             UpStairMode::vw_control(0,-0.02);
             usleep(300000);
             printf("Turn Right\n");
        }
        else
        {
            UpStairMode::robotmotion("Stop");
            printf("OK");
            break;
        }

    }
}
int UpStairMode::mode_21()	//---Alignment & Calculated the tilt angle
{
    /*---Time---*/
    //time_t t1 = time(NULL);
    /*---"depth_find" Function Variable---*/
    int i=0,j=0,x=0,y=0,mid_x=0,mid_y=0,sw,mid_x2=0,mid_y2=0;
    /*---For Alignment(Mode 21) Luck---*/
    int key=0;//Left & Right
    int key1=0;//Far & Close
    int Align=0;
    /*---For 1m Alignment---*/
    //int counter21 = 10000;

    /*---Depth Data Variable---*/
    //int d21=0,cont21=0,i3=0,j3=0,y3=0,sum3=0;
    int cont21=0,i3=0,j3=0,y3=0,sum3=0;
    double xx3=0,t21=0,td21=0;
    float z3=0,x3=0;

    /*---Out While---*/
    int d=800;
    int err=15;
    int far_dis=125,close_dis=115,stop_align_dis=100;
    ros::param::get(NameSpace+"/err", err);
    ros::param::get(NameSpace+"/far_dis", far_dis);
    ros::param::get(NameSpace+"/close_dis", close_dis);
    ros::param::get(NameSpace+"/stop_align_dis", stop_align_dis);
    ros::param::set("current_limit",2000);//expand the current limit
    std::cout<<"err="<<err<<"\n";
    std::cout<<"stop_align_dis="<<stop_align_dis<<"\n";
    /*---Vel - For Record---*/
    int vel_L=0,vel_R=0;
    int cnt=0;
    /*---Data Record - Open---*/
    //    FILE *M2XD=fopen("/home/ubuntu/CodeBlocks/TrackedRobot_Control/Recore Data/M2_XtionDistance.txt","w");
    //    FILE *M2PL=fopen("/home/ubuntu/CodeBlocks/TrackedRobot_Control/Recore Data/M2_PixelLocation.txt","w");
    //    FILE *M2VL=fopen("/home/ubuntu/CodeBlocks/TrackedRobot_Control/Recore Data/M2_VelLeft.txt","w");
    //    FILE *M2VR=fopen("/home/ubuntu/CodeBlocks/TrackedRobot_Control/Recore Data/M2_VelRight.txt","w");
    //    FILE *M2dldr=fopen("/home/ubuntu/CodeBlocks/TrackedRobot_Control/Recore Data/M2_dldr.txt","w");



    printf("Mode21,1m Alignment Mode\n");
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
    //=======================
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

    printf("ZED turn down\n");
    ROS_INFO("bottom:%f,top:%f", bottomm,topp);



    //while(d>600)
    while(ros::ok()&&!safe_switch)
    {

        ros::spinOnce();
        r.sleep();
        mid_x2=0;mid_y2=0;

        printf("d21=%d\n",d21);

        if(abs(d21)<=err)
        {
            printf("counter21 = %d\n",counter21);
            usleep(10000);

            if(counter21<=stop_align_dis) //5F:830, 4F:680
            {
                key1=3;//distance OK
            }

            if(key1!=1 && counter21>stop_align_dis)	//5F:820, 4F:670 ; too far ,TR fwd org=1100  depthMD(mid_x2,mid_y2)>900
            {
                key1=1;
                printf("Too Far\n");
                printf("stop_align_dis=%d\n",stop_align_dis);

                UpStairMode::robotmotion("Stop");
                usleep(5000);
                UpStairMode::robotspeed(350,350);
                usleep(5000);
                UpStairMode::robotmotion("Forward");
                usleep(50000);
            }
            else if(key1!=2 && counter21<close_dis) //5F:780, 4F:590 ; too close,TR back //900 depthMD(mid_x2,mid_y2)<800
            {
                key1=2;
                printf("Too Close\n");
                printf("close_dis=%d\n",close_dis);

                UpStairMode::robotmotion("Stop");
                usleep(5000);
                UpStairMode::robotspeed(350,350);
                UpStairMode::robotmotion("Backward");
                usleep(50000);
            }
            else if(key1==3)
            {
                printf("d21:%d , Mode2 Alignment OK\n",d21);
                UpStairMode::robotmotion("Stop");


                usleep(5000);

                if(sum3!=0) //never happen
                {
                    printf("Calculate the angle\n");
                    z3=float(sum3/j3);
                    //xx3=((((21.2/119)-(21.2/123))/(105.5-101.1))*((depthMD(mid_x2,mid_y2)/10)-101.1)+(21.2/123));
                    x3=abs(mid_y-mid_y2)*xx3;

                    //angg=34;
                    //angg=(57.2958*atan(x3,z3)*6);  //org -> ang=57.2958*atan(x3,z3);
                    //ang=(float)(57.2958*atan(x3,z3)*6);
                    printf("ang=%f \n",angg);


                    /*---Decide to know too left or right---*/
                    t21=abs(i-mid_x2);
                    td21=t21*xx3;
                    printf(" Left side stairs(td21)= %f cm\n",td21);
                    printf(" Left side stairs(mid_x2)= %d cm\n",mid_x2);


                    if(mid_x2<=300) //330
                    {
                        t21=abs(i-mid_x2);
                        td21=t21*xx3;
                        printf(" Left side stairs(td21)= %f cm\n",td21);
                        return 1;//mode_21 failed because of the left side stairs.
                    }
                    //                    else if(mid_x2>=331)
                    //                    {
                    //                        t21=abs(mid_x2-x);
                    //                        td21=t21*xx3;
                    //                        printf("right side stairs(td21)= %f cm\n",td21);
                    //                        return 2;//mode_21 failed because of the right side stairs.
                    //                    }

                    /*---Decide to End mode21---*/
                    if(td21>50 || angg>45 || angg>20) //lsd (left stairs distance)
                    {
                        return 0; //mode_21 succeed.
                    }
                    else return 1;
                }//if(sum3!=0) end
            } //Key3 end
        }//if(d21<=20) end
        //else if(key!=1 && d21>=20 && depthMD(mid_x2-100,mid_y2+20)>depthMD(mid_x2+100,mid_y2+20) && depthMD(mid_x2-100,mid_y2+25)>depthMD(mid_x2+100,mid_y2+25))
        else if( d21<=-err )
        {
            printf("d21:%d , Mode2 Need Left\n",d21);

            UpStairMode::robotmotion("Stop");

            usleep(5000);

            UpStairMode::robotspeed(300,300);

            UpStairMode::robotmotion("Left");

            usleep(300000);

            UpStairMode::robotmotion("Stop");

            usleep(5000);
            key1=0;
        }
        //else if(key!=2 && d21>=20&&depthMD(mid_x2-100,mid_y2+20)<depthMD(mid_x2+100,mid_y2+20) && depthMD(mid_x2-100,mid_y2+25)<depthMD(mid_x2+100,mid_y2+25))
        else if( d21>=err)
        {
            printf("d21:%d , Mode2 Need Right\n",d21);

            UpStairMode::robotmotion("Stop");

            usleep(5000);

            UpStairMode::robotspeed(300,300);

            UpStairMode::robotmotion("Right");

            usleep(300000);//200000

            UpStairMode::robotmotion("Stop");

            usleep(5000);
            key1=0;
        }



        if(counter21<stop_align_dis )
            break;

        ros::spinOnce();
        r.sleep();

    }//while(d>600) end
    // int cnt_dis=0;
    while(ros::ok()&&!safe_switch)
    {
        ros::Rate r(1);
        UpStairMode::robotspeed(350,350);
        if( counter21>far_dis )//80
        {
            UpStairMode::robotmotion("Forward");
            std::cout<<"Too far\n";
            r.sleep();

        }
        else if(counter21<close_dis)//70
        {
            UpStairMode::robotmotion("Backward");
            std::cout<<"Too close\n";
            r.sleep();
        }
        else
        {
            UpStairMode::robotmotion("Stop");
            break;
            //cnt_dis++;
            r.sleep();
        }
        if (counter21>stop_align_dis+10)
        {
            UpStairMode::robotmotion("Stop");
            break;
            //cnt_dis++;
            r.sleep();
        }

        // if(cnt_dis>=3)
        // {
        // 	break;
        // }
    }
    UpStairMode::robotmotion("Stop");
    std::cout<<"The end of Mode 21\n";
    /*---Time end---*/
    //    time_t t2 = time(NULL);
    //    fprintf(M2XD,"Mode2 Time = %d\n",t2-t1);

    /*---Data Record - Close---*/
    //    fclose(M2XD);
    //    fclose(M2PL);
    //    fclose(M2VL);
    //    fclose(M2VR);
    //    fclose(M2dldr);

}//mode21 end

int UpStairMode::mode_22()	//---Alignment & Calculated the tilt angle
{
    int i=0,j=0,x=0,y=0,mid_x=0,mid_y=0,sw,mid_x2=0,mid_y2=0;
    /*---For Alignment(Mode 22) Using PI Control---*/

    float sum_d21=0;
    int d=800;
    int err=15;
    int far_dis=125,close_dis=115,stop_align_dis=100;
    float Kp=1.0,Ki=0.1,vv=0.05;
    ros::param::get(NameSpace+"/err", err);
    ros::param::get(NameSpace+"/far_dis", far_dis);
    ros::param::get(NameSpace+"/close_dis", close_dis);
    ros::param::get(NameSpace+"/stop_align_dis", stop_align_dis);
    ros::param::get(NameSpace+"/vv",vv);
    ros::param::get(NameSpace+"/Kp", Kp);
    ros::param::get(NameSpace+"/Ki", Ki);
    std::cout<<"err="<<err<<"\n";
    std::cout<<"stop_align_dis="<<stop_align_dis<<"\n";
    ros::param::set("current_limit",2000);//expand the current limit

    /*---Vel - For Record---*/
    int vel_L=0,vel_R=0;
    int cnt=0;


    printf("Mode22,PI Alignment Mode\n");
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

    printf("ZED turn down\n");
    ROS_INFO("bottom:%f,top:%f", bottomm,topp);

    UpStairMode::robotmotion("Forward");

    while(counter21>stop_align_dis &&ros::ok()&&!safe_switch)
    {
        if(abs(d21)>1000)
            d21=0;

        sum_d21=sum_d21+d21;
        if(counter21>far_dis)
        {
            UpStairMode::vw_control(vv,-d21*Kp-sum_d21*Ki);
            std::cout<<"v="<<vv<<",w="<<-d21*Kp-sum_d21*Ki<<"\n";
            usleep(100000);

        }
        else if(counter21<close_dis)
        {
            UpStairMode::vw_control(-vv,-d21*Kp-sum_d21*Ki);
            std::cout<<"v="<<vv<<",w="<<-d21*Kp-sum_d21*Ki<<"\n";
            usleep(100000);

        }
        std::cout<<"d21="<<d21<<",sumd21="<<sum_d21<<"\n";
    }
    std::cout<<"stop alignment\n";
    while(ros::ok()&&!safe_switch){

        if (counter21>far_dis)
            UpStairMode::vw_control(0.05,0); //too far
        else if(counter21<close_dis)
            UpStairMode::vw_control(-0.05,0); //too close
        else if(counter21>stop_align_dis+10)
            UpStairMode::vw_control(-0.05,0); //camera is not accurate or too close
        else
        {
            UpStairMode::vw_control(0,0);//stop
            std::cout<<"mode22 end!\n";
            break;
        }
        usleep(100000);
    }
    UpStairMode::robotmotion("Stop");
    usleep(100000);
    //        v=0.1 w=0

}//mode22 end
void UpStairMode:: stair_alignment(double err)
{
    double d23=laser_170-laser_340;
    double Lstop_align_dis =5;
    int con=0;
    std::cout<<"err="<<err<<"\n";
    while(ros::ok()&&!safe_switch)
    {
        ros::spinOnce();
        if(abs(laser_170)>5)
            laser_170=Lstop_align_dis;
        if(abs(laser_340)>5)
            laser_340=Lstop_align_dis;
        if(std::isnan(laser_340))
            laser_340=0;
        if(std::isnan(laser_170))
            laser_170=0;


//         if(abs(laser_260)>5)
//         {
//             UpStairMode::vw_control(0.08,0);
//         }
//         else
//         {
//             UpStairMode::vw_control(0,0);
//             con=0;
//         }
         usleep(100000);

         d23=laser_170-laser_340;

         std::cout << "  laser170 = "<<laser_170<< std::endl;
         std::cout << "  laser340 = "<<laser_340<< std::endl;
         std::cout << "  laser260 = "<<laser_260<< std::endl;

         if(d23>err)//too right
         {
             if(con==1)
                 continue;
             con=1;
             UpStairMode::vw_control(0,0.02);
         }
         else if(d23<-err)//too left
         {
             if(con==2)
                 continue;
             con=2;
             UpStairMode::vw_control(0,-0.02);
         }
         else
         {
             UpStairMode::vw_control(0,0);
             usleep(100000);
             break;
         }
         usleep(100000);

         std::cout<<"d23="<<d23<<"\n";
    }

    std::cout << "  laser170 = "<<laser_170<< std::endl;
    std::cout << "  laser340 = "<<laser_340<< std::endl;
    std::cout << "  laser260 = "<<laser_260<< std::endl;
    UpStairMode::vw_control(0,0);
    usleep(100000);
    std::cout<<"stop alignment\n";
}
int UpStairMode::mode_23(double vv,double Lfar_dis,double Lclose_dis,double Lstop_align_dis,bool leglock)	//---Alignment with laser & Calculated the tilt angle
{
    int i=0,j=0,x=0,y=0,mid_x=0,mid_y=0,sw,mid_x2=0,mid_y2=0;
    /*---For Alignment(Mode 22) Using PI Control---*/

    double sum_d23=0;
    int d=800;
    int err=15;



    //double Lfar_dis=0.57,Lclose_dis=0.45,Lstop_align_dis=1;
    float Kp=0.1,Ki=0.0001;
    ros::param::get(NameSpace+"/Kp", Kp);
    ros::param::get(NameSpace+"/Ki", Ki);
    ros::param::get(NameSpace+"/err", err);
    ros::param::get(NameSpace+"/Lfar_dis", Lfar_dis);
    ros::param::get(NameSpace+"/Lclose_dis", Lclose_dis);
    ros::param::get(NameSpace+"/Lstop_align_dis", Lstop_align_dis);
    ros::param::set("current_limit",10000);//expand the current limit
    //    ros::param::get(NameSpace+"/vv", vv);

    //    std::cout<<"err="<<err<<"\n";
    //    std::cout<<"stop_align_dis="<<stop_align_dis<<"\n";

    /*---Vel - For Record---*/
    int vel_L=0,vel_R=0;
    int cnt=0;


    printf("Mode23,Lidar Alignment Mode\n");
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

    bottomm=bottom;
    topp=top;
    //    front_dd=front_d-10;
    joint_command.request.unit = "rad";
    joint_command.request.id = 2;
    joint_command.request.goal_position = bottomm*PI/180;
    joint_command_client.call(joint_command);
    joint_command.request.unit = "rad";
    joint_command.request.id = 1;
    joint_command.request.goal_position = topp*PI/180;
    joint_command_client.call(joint_command);
    joint_command.request.unit = "rad";
    //    joint_command.request.id = 3;
    //    joint_command.request.goal_position = front_dd*PI/180;
    //    joint_command_client.call(joint_command);

    printf("ZED turn down\n");
    ROS_INFO("bottom:%f,top:%f", bottomm,topp);

//    UpStairMode::leghome(0,0);
    UpStairMode::robotmotion("Forward");

//    while(laser_180>Lstop_align_dis &&ros::ok()&&!safe_switch)
//    {
//        if(std::isnan(sum_d23))
//            sum_d23=0;

//        if(std::isinf(laser_180)||std::isnan(laser_180))//||std::isinf(laser_340)||std::isnan(laser_340))
//        {
//            laser_170=Lstop_align_dis+1;
////            laser_340=Lstop_align_dis+1;
//            sum_d23=0;
//        }
//        if (abs(laser_180)>10)
//            laser_180=Lstop_align_dis+1;
////        if(abs(laser_170)>10)
////            laser_170=Lstop_align_dis+1;
////        if(abs(laser_340)>10)
////            laser_340=Lstop_align_dis+1;
////        if(abs(laser_260)>10)
////            laser_260=Lstop_align_dis+1;


//        if(abs(d23)>1)
//            d23=0;
//        std::cout << "  laser150 = "<<laser_150<< std::endl;
//        std::cout << "  laser180 = "<<laser_180<<"   <-------"<<std::endl;
//        std::cout << "  laser210 = "<<laser_210<< std::endl;
////        sum_d23=sum_d23+d23;
////        if(laser_260>2*Lfar_dis)
////        {
////            if(d23>0.1)//too right
////                UpStairMode::vw_control(vv,0.02);
////            else if(d23<-0.05)//too left
////                UpStairMode::vw_control(vv,-0.02);
////            else
////                UpStairMode::vw_control(vv,0);
////            sleep(1);
////        }
////        else if(laser_260>Lfar_dis)
////        {
////            if(d23>0.1)//too right
////                UpStairMode::vw_control(vv,0.02);
////            else if(d23<-0.05)//too left
////                UpStairMode::vw_control(vv,-0.02);
////            else
////                UpStairMode::vw_control(vv,0);
////            sleep(1);
////        }
////        else if(laser_260<Lclose_dis)
////        {
////            if(d21>0.1)//too right
////                UpStairMode::vw_control(-vv,0.02);
////            else if(d21<-0.05)//too left
////                UpStairMode::vw_control(-vv,-0.02);
////            else
////                UpStairMode::vw_control(-vv,0);
////            sleep(1);
////        }
//        //        if(laser_260>Lfar_dis)
//        //        {
//        //            UpStairMode::vw_control(vv,d23*Kp);//d23*Kp+sum_d23*Ki
//        //            std::cout<<"v="<<vv<<",w="<<d23*Kp<<"\n";/*+sum_d23*Ki*/
//        //             sleep(1);

//        //        }
//        //        else if(laser_260<Lclose_dis)
//        //        {
//        //            UpStairMode::vw_control(-vv,d23*Kp);
//        //            std::cout<<"v="<<vv<<",w="<<d23*Kp<<"\n";
//        //            sleep(1);

//        //        }
//        std::cout<<"d23="<<d23<<",sumd23="<<sum_d23<<"\n";
//    }
//    UpStairMode::robotspeed(0,0);
//    std::cout<<"stop alignment\n";
//    sleep(2);
    //    front_dd=front_d;
    //    joint_command.request.unit = "rad";
    //    joint_command.request.id = 3;
    //    joint_command.request.goal_position = front_dd*PI/180;
    //    joint_command_client.call(joint_command);
    //    sleep(0.5);
    int align = 0;
    while(ros::ok()&&!safe_switch)
    {

        printf("laser180 : %f     <<<<<------\n", laser_180);

        if(std::isinf(laser_180))
        {
            UpStairMode::robotmotion("Stop");
            usleep(300000);
            printf("Blind !!\n");
        }

        else if ( laser_180 > Lfar_dis)
        {
            UpStairMode::robotspeed(350,350); //too far   --> need to drive forward
            UpStairMode::robotmotion("Forward");
            printf("forward\n");
            usleep(300000);
        }
//        else if(laser_180<=Lfar_dis-0.02)
//        {
//            UpStairMode::robotspeed(100,100);//too close  --> need to drive backwards
//            UpStairMode::robotmotion("Backward");
//            printf("backwards\n");
//            sleep(1);
//        }
        else                                 //      ok   --> start alignment then put down arms
        {
            if(align>0)
            {
                break;
            }
            UpStairMode::robotmotion("Stop");
            usleep(300000);
            printf("Stop!\n");
            printf("Start alignment ...\n");
            sum_d23 = 0;
            while(ros::ok()&&!safe_switch)
            {
                if(!((std::isinf(laser_150))||(std::isinf(laser_210))))
                {
                    double d23=laser_150-laser_210;
                    printf("d23 = %lf ",d23);

                    if( d23 < -0.01 )//  need to turn right
                    {
                        printf("\t-->\tTurn Right\n");
                        UpStairMode::vw_control(0,-0.02);
                        usleep(300000);
                        sum_d23=0;
                    }
                    else if( d23 > 0.035 )// need to turn left
                    {
                        printf("\t-->\tTurn Left\n");
                        UpStairMode::vw_control(0,0.02);
                        usleep(300000);
                        sum_d23=0;
                    }
                    else
                    {
                        printf("\t-->\tOk!\n");
                        UpStairMode::robotmotion("Stop");
                        usleep(300000);
                        printf("sum_d23 = %lf \n",sum_d23);
                        sum_d23++;
                        if(sum_d23==3)
                        {
                            align ++;
                            std::cout<<"stop alignment\n";
                            break;

                        }
                    }
                }
                else
                {
                    UpStairMode::vw_control(0,0);
                    usleep(300000);
                }
            }
        }
//        else if(laser_260<Lclose_dis)
//            UpStairMode::vw_control(-0.05,0); //too close
        /*else if(counter21>stop_align_dis+10)
            UpStairMode::vw_control(-0.05,0); //camera is not accurate or too close*/
//        else
//        {
//            UpStairMode::vw_control(0,0);//stop
//            sleep(1);
//            if (!leglock)
//            {
//                std::cout<<"!! Ready to put down arms!!\n";
//                sleep(1);
//                UpStairMode::vw_control(0,0);
//                UpStairMode::legMA(8691,9785);
//                sleep(1);
//                UpStairMode::legMA_feedback(8691,9785);
//            }
//            std::cout<<"mode23 end!\n";
//            break;
//        }
    }
    UpStairMode::robotspeed(100,100);
    UpStairMode::robotmotion("Forward");
    usleep(300000);
    while(ros::ok())
    {
        ros::spinOnce();
        printf("%f      ",laser_180);
        if(laser_180 < 0.40)
        {
            UpStairMode::robotmotion("Stop");
            usleep(300000);
            printf("Close enough!! Stop!!\n");
            break;
        }
        else
        {
            printf("Keep Forward !!\n");
        }
    }

    if (!leglock)
    {
        std::cout<<"!! Ready to put down arms!!\n";
        UpStairMode::legspeed(150,100);
        usleep(300000);
        UpStairMode::vw_control(0,0);
        UpStairMode::legMA(23000,15000);
        for( int i = 1 ; i <= 25 ; i++ )
        {
            topp  = 2*i;
            joint_command.request.unit = "rad";
            joint_command.request.id = 1;
            joint_command.request.goal_position = topp*PI/180;
            joint_command_client.call(joint_command);
            usleep(10);
        }
        UpStairMode::legMA_feedback(20000,15000);
        usleep(300000);
    }

    std::cout<<"mode23 end!\n";
    UpStairMode::robotmotion("Stop");
//    sleep(0.01);
    //        v=0.1 w=0

}//mode22 end
void UpStairMode::mode_3()
{
    /*---Time---*/
    //time_t t1 = time(NULL);

    /*---Data Record - open---*/
    //FILE *M3Time=fopen("/home/ubuntu/CodeBlocks/TrackedRobot_Control/Recore Data/M3_time.txt","w");


    std_msgs::Int32MultiArray robot_speed,leg_speed,robot_MA,leg_MA,robot_HO,robot_VA,robot_MR,leg_HO,leg_VA;

    std_msgs::Int32 robot_motion,leg_motion;

    dynamixel_workbench_msgs::JointCommand joint_command;
    ros::param::set("current_limit",10000);

    robot_speed.data.clear();
    leg_speed.data.clear();
    robot_MA.data.clear();
    leg_MA.data.clear();
    robot_HO.data.clear();
    leg_HO.data.clear();
    robot_VA.data.clear();
    robot_MR.data.clear();

    ros::Rate r(1);

    ros::spinOnce();
    usleep(300000);
    UpStairMode::robotmotion("Stop");

    usleep(300000);

    //XtionMotor(740-angg,0); //375 5F 640

    bottomm=bottom;
    topp=top;
    joint_command.request.unit = "rad";
    joint_command.request.id = 2;
    joint_command.request.goal_position = bottomm*PI/180;
    joint_command_client.call(joint_command);
//    joint_command.request.unit = "rad";
//    joint_command.request.id = 2;align_depth
//    joint_command.request.goal_position = topp*PI/180;
//    joint_command_client.call(joint_command);

    printf("ZED turn down\n");
    ROS_INFO("bottom:%f,top:%f", bottomm,topp);

    usleep(500000);

    printf("Mode 3 , Up Stairs Prepare\n");

    if(angg<45 && angg>20)
    {

        UpStairMode::robotmotion("Stop");
        usleep(300000);

        UpStairMode::robotspeed(400,400);

        usleep(300000);

        UpStairMode::legspeed(100,100);

        usleep(300000);


        UpStairMode::robotmotion("Forward");

        usleep(300000);

//        UpStairMode::robotspeed(0,0);


//        UpStairMode::legMA(5070,9785);
//        UpStairMode::legMA(300000,400000);

//         sleep(5);

         UpStairMode::legspeed(120,70);
         usleep(300000);

         UpStairMode::legMA_feedback(900,-2000);
         usleep(300000);
         UpStairMode::robotmotion("Stop");
         usleep(300000);

         UpStairMode::robotmotion("Forward");
         sleep(3);
         //UpStairMode::legMA_feedback(900,-2000);

         //usleep(300000);
         //UpStairMode::legMA_feedback(-495,-2000);

         UpStairMode::robotmotion("Stop");
         usleep(300000);
//         //		usleep(2500000);

// //        sleep(8);

//         //        UpStairMode::legMA(480000,510000);
//         //        sleep(4);

//         UpStairMode::robotmotion("Stop");

//         usleep(500000);

//         //        front_dd=front_d-50;490
//         //        joint_command.request.unit = "rad";
//         //        joint_command.request.id = 3;
//         //        joint_command.request.goal_position = front_dd*PI/180;
//         //        joint_command_client.call(joint_command);

//=====================================
        //front leg put on stair
//=====================================
        // UpStairMode::legMA(30000,39490);

//        usleep(1000000);

//        UpStairMode::motor_switch(1,0);
//        usleep(1500000);
//        UpStairMode::motor_switch(1,1);
//        usleep(500000);

        printf("mode3 End\n");
//        bottomm=bottom;
//        topp=top-50;
//        joint_command.request.unit = "rad";
//        joint_command.request.id = 1;
//        joint_command.request.goal_position = bottomm*PI/180;
//        joint_command_client.call(joint_command);
//        joint_command.request.unit = "rad";
//        joint_command.request.id = 2;
//        joint_command.request.goal_position = topp*PI/180;
//        joint_command_client.call(joint_command);
    }
    else
    {
        printf("Mode3 Stair Angle Greater or less than Limit\n");

    }



}//mode3 end

void UpStairMode::mode_4()	//---Climbing mode
{
    std_msgs::Int32MultiArray robot_speed,leg_speed,robot_MA,leg_MA,robot_HO,robot_VA,robot_MR,leg_HO,leg_VA;

    std_msgs::Int32 robot_motion,leg_motion;
    dynamixel_workbench_msgs::JointCommand joint_command;

    int md21_max=170,d21_high=30,d21_low=-30;
    double top_laser = 2.5;
    int left_enc_old=0,right_enc_old=0;
    int counter = 0 ;
    //    ros::param::get(NameSpace+"/top_laser", top_laser);
    ros::param::get(NameSpace+"/md21_max", md21_max);
    ros::param::get(NameSpace+"/d21_high", d21_high);
    ros::param::get(NameSpace+"/d21_low", d21_low);

    ros::param::set("current_limit",10000);//reset the current limit

    robot_speed.data.clear();
    leg_speed.data.clear();
    robot_MA.data.clear();
    leg_MA.data.clear();
    leg_VA.data.clear();
    robot_HO.data.clear();
    robot_VA.data.clear();
    robot_MR.data.clear();
    leg_HO.data.clear();

    ros::Rate r(10);

    //ros::spinOnce();
    //r.sleep();
//    int stuck_count=0;
//    int cont4=0;
//    int key=0;
//    int r_cont4=0;
//    int vel_L=0,vel_R=0;
//    double old_pitch=0;
    bool pass = false;




    printf("Mode 4 , Climbing Mode\n");



    UpStairMode::legspeed(100,100);
    usleep(300000);
//    UpStairMode::robotmotion("Stop");
//    usleep(500000);

    UpStairMode::robotspeed(350,350);//at least upstair for 3 sec to prevent shaking depth error

    usleep(300000);
//    UpStairMode::vw_control(0.15,0);
//    usleep(8000000);
//    printf("Start to Align");
//    while(ros::ok()&&!safe_switch)
//    {
//        if(!((std::isinf(laser_225))||(std::isinf(laser_270)))) //if the robot isn't going straight
//        {
//            align_laser=laser_225-laser_270;
//            if( align_laser>=0.4 )//  need to turn right
//            {
//                UpStairMode::vw_control(0.05,-0.01);
//                usleep(300000);
//                printf("Turn Right\n");
//            }
//            else if( align_laser<=0.2)// need to turn left
//            {
//                UpStairMode::vw_control(0.05,0.01);
//                usleep(300000);
//                 printf("Turn left\n");
//            }
//            else if (align_laser>=0.31 || align_laser<=0.2)
//            {
//                UpStairMode::vw_control( 0.1, 0);
//                printf("Go Straight\n");
//                break;
//                }
//            }
//     }
//    sleep(1);
    printf("ZED turn down\n");
    //    printf("Hokuyo turn down 50 degree \n");

    bottomm=bottom;
    topp=top+50;
    //    front_dd=front_d-45;
    joint_command.request.unit = "rad";
    joint_command.request.id = 2;
    joint_command.request.goal_position = bottomm*PI/180;
    joint_command_client.call(joint_command);
    joint_command.request.unit = "rad";
    joint_command.request.id = 1;
    joint_command.request.goal_position = topp*PI/180;
    joint_command_client.call(joint_command);


    printf("------ Realsense Camera Ready ! ------\n");
//    sleep(1);
//    usleep(300000);
    //    joint_command.request.unit = "rad";
    //    joint_command.request.id = 3;
    //    joint_command.request.goal_position = front_dd*PI/180;
    //    joint_command_client.call(joint_command);

    //    while(ros::ok()&&!safe_switch) //while1 : climbing mission restart.
    //    {
    UpStairMode::robotspeed(1000,1000);
    UpStairMode::robotmotion("Stop");
    usleep(300000);
    UpStairMode::robotmotion("Forward");
    while(ros::ok()&&!safe_switch) //while2 : climbing stair
    {



        //            printf("ultrasonic=%d\n",ultrasonic);
        //            std::cout<<"laser_260="<<laser_260<<"\n";

        ros::spinOnce();
//        printf("mid:(%d,%d) , d4:%d , cnt21:%d\n",mid_x,mid_y,d4,counter21);
//        printf("MD21=%d\n",MD21);

        //------ Stop if it reaches the top ------//
        if( (UpStairMode::depth_scan_hit(95,90)))
        {
            UpStairMode::robotmotion("Stop");
            printf("Ready to put down arm!!");
            usleep(300000);
            break;
        }
//        else if (counter<5)
//            {
//                printf("Stair distance Align!!/n");
//                if(laser_270<0.3) //too close to wall turn left
//                {
//                    printf("Too close! Turn left!/n");
//                    UpStairMode::vw_control(0.05,0.04);
//                    sleep(1);
//                    UpStairMode::vw_control(0.05,-0.04);
//                    sleep(1);
//                    printf("counter=%d",counter);
//                    counter++;
//                }
//                else if(laser_270>0.3)  // too far from the wall turn right
//                {
//                    printf("Too far! Turn right!/n");
//                    UpStairMode::vw_control(0.05,-0.04);
//                    sleep(1);
//                    UpStairMode::vw_control(0.05,0.04);
//                    sleep(1);
//                    printf("counter=%d",counter);
//                    counter++;
//                }
//                else
//                {
//                    printf("Go straignt!/n");
//                    UpStairMode::vw_control(0.05,0);
//                    printf("counter=%d",counter);
//                    counter ++;
//                    sleep(1);
//                }
//            }
        else
        {
            printf("Climbling Align");

            if(!((std::isinf(laser_225))||(std::isinf(laser_270)))) //if the robot isn't going straight and laser ins't nan
            {
                align_laser=laser_225-laser_270; //laser225=-45 laser270=-90
                if( align_laser>=0.4 )//  need to turn right
                {
                    UpStairMode::vw_control(0.1,-0.02);
                    usleep(300000);
                    printf("\t-->\tTurn Right\n");
                }
                else if( align_laser<=0.35)// need to turn left
                {
                    UpStairMode::vw_control(0.1,0.02);
                     usleep(300000);
                     printf("\t-->\tTurn left\n");
                }
                else
                {
                    UpStairMode::vw_control( 0.1, 0);
                    usleep(300000);
                    printf("\t-->\tGo Straight\n");
                }
            }

            else
            {
                printf("\t-->\t Invalid Data !! Go Straight\n");
                UpStairMode::vw_control( 0.1, 0);
                usleep(300000);
            }
        }


        //------Align the Route if it is still climbing ------//
        //        sleep(0.1);
//        if(  MD21<=md21_max && ros::ok() &&!safe_switch)//laser_260 < top_laser&& ultrasonic<=100 mid_x!=0 && mrid_y!=0&&MD21<=md21_max  MD21<=md21_max && ros::ok()
//        {
//            if(r_cont4>0)
//            {
//                UpStairMode::robotmotion("Forward");

//                usleep(500000);
//                r_cont4--;
//                cont4=0;
//            }
//            else if(d21<=d21_high && d21>=d21_low ) //4
//            {

//                UpStairMode::robotspeed(1350,1350);
//                usleep(500000);

//                UpStairMode::robotmotion("Forward");

//                usleep(500000);
//                cont4=0;
//            }
//            else if(d21>d21_high)
//            {
//                printf("need turn left\n");
//                UpStairMode::robotspeed(1000,1350);

//                usleep(500000);

//                UpStairMode::robotmotion("Forward");

//                usleep(500000);
//                cont4=0;

//            }
//            else if(d21<d21_low)
//            {
//                printf("need turn right\n");

//                UpStairMode::robotspeed(1350,1000);

//                usleep(500000);

//                UpStairMode::robotmotion("Forward");

//                usleep(500000);
//                cont4=0;
//            }

//            //            cont4=0;
//        }
        //            else if(left_enc_old==Encoders[1]||right_enc_old==Encoders[2])
        //            {
        //                printf("Detect teeeeracked robot is stuck\n");
        //                printf("counter = %d/n",stuck_count);
        //                if(stuck_count>=3)break;
        //            }
//        else
//        {
//            printf("---Ready go to the top!---\n");
////            sleep(0.1);
//            UpStairMode::robotmotion("Stop");
//            sleep(1);
//            printf("Ready to put down arm!!");
//            cont4++;
//            r_cont4++;
////            UpStairMode::robotspeed(350,350);
////            usleep(500000);
////            std::cout<<"cont4="<<cont4<<std::endl;
//            if(cont4>=3)
//                break;
//        }

//        if(abs(pitch-old_pitch)>10&& ros::ok() &&!safe_switch && old_pitch!=0)
//        {
//            printf("The robot already land!!");
//            pass=true;
//            break;
//        }
//        old_pitch=pitch;

    }//while 2 end

    //        if(stuck_count>=3)//The climbing mission is fail,moving back to restart.
    //        {
    //            printf("The climbing mission is fail,moving back to restart. \n");
    //            UpStairMode::robotspeed(500,500);
    //            usleep(50000);
    //            UpStairMode::robotmotion("Backward");
    //            sleep(5);
    //            UpStairMode::robotmotion("Stop");
    //            usleep(50000);
    //        }

    //        else
    //        {
    //            printf("---Near to TOP---\n\n\n");
    //            UpStairMode::legspeed(2000,2000);
    //            r.sleep();
    //            break;
    //        }// while 1 end
    //    }

    printf("---TOP Lay Down Front Arm---\n\n\n");

    /*---02.Stop and set speed---*/


//    sleep(0.1);
    bottomm=bottom;
    topp=50;
    //    front_dd=front_d;
//    joint_command.request.unit = "rad";
//    joint_command.request.id = 1;
//    joint_command.request.goal_position = bottomm*PI/180;
//    joint_command_client.call(joint_command);
//    joint_command.request.unit = "rad";
//    joint_command.request.id = 2;
//    joint_command.request.goal_position = topp*PI/180;
//    joint_command_client.call(joint_command);
    //    joint_command.request.unit = "rad";
    //    joint_command.request.id = 3;
    //    joint_command.request.goal_position = front_dd*PI/180;
    //    joint_command_client.call(joint_command);

    printf("ZED init\n");

//    if(!pass)
//    {
//        usleep(300000);

//        UpStairMode::robotspeed(100,100);
//        usleep(100000);

//        UpStairMode::robotmotion("Stop");
//        sleep(2);
//        UpStairMode::legMA(-46000,64000);
//        sleep(2);
//        UpStairMode::legspeed(100,100);
//        usleep(500000);
//        UpStairMode::legMA(54000,64000);
//        sleep(1);
//        UpStairMode::robotmotion("Forward");
//        sleep(7);
//        UpStairMode::legspeed(100,100);
//        usleep(500000);
//        UpStairMode::legMA(0,-56000);
//        sleep(2);
//    }


    //	/*---03---*/
    usleep(300000);
    UpStairMode::robotmotion("Stop");
    usleep(300000);
    //sleep(300);
    UpStairMode::legspeed(100,100);
    UpStairMode::robotspeed(350,350);
    usleep(300000);
    UpStairMode::robotmotion("Forward");
    UpStairMode::legMA_feedback(-6500,-10000);
    usleep(300000);
    UpStairMode::legspeed(200,200);
    UpStairMode::legMA_feedback(100,100);
    sleep(6);
    UpStairMode::robotmotion("Stop");
    usleep(300000);

    for(int i = 1;i<=4;i++)
    {
        UpStairMode::motor_switch(i,0);
        usleep(300000);
    }
    sleep(2);

    for(int i = 1;i<=4;i++)
    {
        UpStairMode::motor_switch(i,1);
        usleep(300000);
    }
    sleep(2);

//    UpStairMode::motor_switch(1,0);
//    UpStairMode::motor_switch(2,0);
//    UpStairMode::motor_switch(3,0);
//    UpStairMode::motor_switch(4,0);
//    usleep(300000);
//    UpStairMode::motor_switch(1,1);
//    UpStairMode::motor_switch(2,1);
//    UpStairMode::motor_switch(3,1);
//    UpStairMode::motor_switch(4,1);
//    usleep(300000);
    UpStairMode::robotspeed(350,350);
    UpStairMode::legspeed(300,230);
    UpStairMode::legMA(38000,29000);
    usleep(300000);

    joint_command.request.unit = "rad";
    joint_command.request.id = 2;
    joint_command.request.goal_position = bottomm*PI/180;
    joint_command_client.call(joint_command);

    for(int i = 50 ; i <= 0 ; i--)
    {
        topp = i;
        joint_command.request.unit = "rad";
        joint_command.request.id = 1;
        joint_command.request.goal_position = topp*PI/180;
        joint_command_client.call(joint_command);
        usleep(100);
    }
    UpStairMode::robotmotion("Stop");
    UpStairMode::legMA_feedback(38000,29000);
    usleep(300000);
    UpStairMode::robotmotion("Stop");

    printf("---Already On the TOP---\n");
    ros::param::set("current_limit",10000);//reset the current limit

    printf("Mode4 End \n\n\n");

}	//mode4 end


void UpStairMode::mode_41(std::string direction,int angle_zed)	//---Climbing mode
{
    std_msgs::Int32MultiArray robot_speed,leg_speed,robot_MA,leg_MA,robot_HO,robot_VA,robot_MR,leg_HO,leg_VA;

    std_msgs::Int32 robot_motion,leg_motion;

    dynamixel_workbench_msgs::JointCommand joint_command;

    int md21_max=170,d21_high=30,d21_low=-30;
    double top_laser = 2.5;
    int left_enc_old=0,right_enc_old=0;
    //    ros::param::get(NameSpace+"/top_laser", top_laser);
    ros::param::get(NameSpace+"/md21_max", md21_max);

    robot_speed.data.clear();
    leg_speed.data.clear();
    robot_MA.data.clear();
    leg_MA.data.clear();
    leg_VA.data.clear();
    robot_HO.data.clear();
    robot_VA.data.clear();
    robot_MR.data.clear();
    leg_HO.data.clear();

    ros::Rate r(10);

    //ros::spinOnce();
    //r.sleep();
    int stuck_count=0;
    int cont4=0;
    int key=0;
    int r_cont4=0;
    int vel_L=0,vel_R=0,vel_c=1350;
    double old_pitch=0;
    bool pass = false;
    double Las_err=0,Las_0=0,Las_30=0;
    int speed=1350,condition=0;

    printf("Mode 4 , Climbing Mode\n");



    UpStairMode::legspeed(1000,1000);
    usleep(500000);
    UpStairMode::robotmotion("Stop");
    usleep(500000);

    UpStairMode::robotspeed(1350,1350);//at least upstair for 3 sec to prevent shaking depth error
    usleep(500000);
    UpStairMode::robotmotion("Forward");
    sleep(1);
    printf("ZED turn down\n");
    //    printf("Hokuyo turn down to 40\n");

    bottomm=bottom;
    topp=top+angle_zed;
    //    front_dd=front_d-45;
    joint_command.request.unit = "rad";
    joint_command.request.id = 1;
    joint_command.request.goal_position = bottomm*PI/180;
    joint_command_client.call(joint_command);
    joint_command.request.unit = "rad";
    joint_command.request.id = 2;
    joint_command.request.goal_position = topp*PI/180;
    joint_command_client.call(joint_command);

    sleep(0.1);
    ros::spinOnce();
    while(ros::ok()&&!safe_switch) //while2 : climbing stair
    {

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
            break;
        }
        ros::spinOnce();

        printf("MD21=%d\n",MD21);
        std::cout<<"Las_err="<<Las_err<<"\n";
        std::cout<<"Las_0="<<Las_0<<"\n";
        std::cout<<"Las_30="<<Las_30<<"\n";
        std::cout<<"pitch error ="<<abs(pitch-old_pitch)<<"\n";
        if(  MD21<=md21_max && ros::ok() &&!safe_switch)//laser_260 < top_laser&& ultrasonic<=100 mid_x!=0 && mrid_y!=0&&MD21<=md21_max  MD21<=md21_max && ros::ok()
        {

            if(MD21>=md21_max/2)
                speed=800;
            else
                speed=1350;
            if(r_cont4>0)
            {
                UpStairMode::robotmotion("Forward");
                usleep(500000);
                r_cont4--;
                cont4=0;
            }
            else if(Las_err<0) //approach wall
            {
                if(Las_0>1 && Las_30>1){//go straight
                    if(condition==1)
                    condition=1;
                    UpStairMode::robotspeed(speed,speed);
                    usleep(500000);
                }
                else{//go away
                    if(condition==2)
                    condition=2;
                    UpStairMode::robotspeed(speed+vel_L,speed+vel_R);
                    usleep(500000);
                }
                UpStairMode::robotmotion("Forward");
                usleep(500000);
                cont4=0;
            }
            else if(Las_err>0.02)//keep away from wall
            {
                if(Las_0<0.8 && Las_30<0.8){//go straight
                    if(condition==1)
                    condition=1;
                    UpStairMode::robotspeed(speed,speed);
                    usleep(500000);
                }
                else    //appraoch the wall
                {
                    if(condition==3)
                    condition=3;
                    UpStairMode::robotspeed(speed-vel_L,speed-vel_R);
                    usleep(500000);
                }

                UpStairMode::robotmotion("Forward");
                usleep(500000);
                cont4=0;

            }
            else
            {
                if(Las_0>1)
                {
                    if(condition==3)
                    condition=3;
                    UpStairMode::robotspeed(speed-vel_L,speed-vel_R);//approach
                    usleep(500000);
                }
                else if(Las_0<0.8)
                {
                    if(condition==2)
                    condition=2;
                    UpStairMode::robotspeed(speed+vel_L,speed+vel_R);//go away
                    usleep(500000);
                }
                else
                {
                    if(condition==1)
                    condition=1;
                    UpStairMode::robotspeed(speed,speed);//go straight
                    usleep(500000);
                }
                UpStairMode::robotmotion("Forward");
                usleep(500000);
                cont4=0;
            }

        }

        else
        {
            printf("---Ready go to the top!---\n");

            UpStairMode::robotmotion("Stop");
            sleep(1);
            cont4++;
            r_cont4++;
            UpStairMode::robotspeed(350,350);
            usleep(500000);
            std::cout<<"cont4="<<cont4<<std::endl;
            if(cont4>=2)break;
        }

        if(abs(pitch-old_pitch)>10&& ros::ok() &&!safe_switch && old_pitch!=0)
        {
            printf("The robot already land!!");
            pass=true;
            break;
        }
        old_pitch=pitch;

    }

    printf("---TOP Lay Down Front Arm---\n\n\n");

    /*---02.Stop and set speed---*/


    sleep(0.1);
    bottomm=bottom;
    topp=top;
    //    front_dd=front_d;
    joint_command.request.unit = "rad";
    joint_command.request.id = 1;
    joint_command.request.goal_position = bottomm*PI/180;
    joint_command_client.call(joint_command);
    joint_command.request.unit = "rad";
    joint_command.request.id = 2;
    joint_command.request.goal_position = topp*PI/180;
    joint_command_client.call(joint_command);

    printf("ZED init\n");

    if(!pass)
    {

//        if(direction=="Left")
//            UpStairMode::wall_alignment_l(0,0.03);
//        else if(direction =="Right")
//            UpStairMode::wall_alignment_r(0,0.03);

//        usleep(500000);

//        UpStairMode::robotspeed(900,900);
//        usleep(100000);

//        UpStairMode::robotmotion("Stop");
//        sleep(1);
//        UpStairMode::legMA(390000,490000);
//        sleep(2);
//        UpStairMode::robotmotion("Forward");
//        sleep(4);
//        UpStairMode::legMA(450000,490000);
//        sleep(3);
//        UpStairMode::legMA(370000,370000);
//        sleep(3);
        usleep(500000);

        UpStairMode::robotspeed(900,900);
        usleep(100000);

        UpStairMode::robotmotion("Stop");
        sleep(1);
        UpStairMode::legMA(-46000,64000);
        sleep(2);
        UpStairMode::legMA(134000,64000);
        sleep(1);
        UpStairMode::robotmotion("Forward");
        sleep(5);
        UpStairMode::legMA_feedback(0,0);
    }

    //	/*---04---*/

    UpStairMode::robotmotion("Stop");
    usleep(50000);
    printf("---Already On the TOP---\n");
    ros::param::set("current_limit",1000);//reset the current limit

    printf("Mode4 End \n\n\n");

}	//mode4 end

double UpStairMode::Align180()
{
    return laser_135-laser_90;
}

void UpStairMode::lasercallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    /*    double min,max;
    min = msg->angle_min;
    max = msg->angle_max;
    std::cout<<"min angle="<<min/PI*180<<" max angle="<<max/PI*180<<std::endl*/;
    laser_2 = msg->ranges[90];
    laser_32 = msg->ranges[120];
    laser_170 = msg->ranges[150];
    laser_340 = msg->ranges[180];
//    laser_150 = msg->ranges[210];
    laser_481 = msg->ranges[240];
    laser_511 = msg->ranges[270];
    laser_180 = msg->ranges[257];
    laser_150 = msg->ranges[171];
    laser_210 = msg->ranges[343];
    laser_225 = msg->ranges[129];
    laser_270 = msg->ranges[0];
    laser_135=msg->ranges[386];
    laser_90=msg->ranges[512];
//        std::cout << "  lidar150 = "<<laser_170<< std::endl;
//        std::cout << "  lidar180 = "<<laser_340<<"   <<<--"<< std::endl;
//        std::cout << "  lidar210 = "<<laser_260<< std::endl;

}
double UpStairMode::getlaser(int number)
{
    ros::spinOnce();
    usleep(500000);
    switch(number)
    {
        case 2:
            return laser_2;
        case 32:
            return laser_32;
        case 170:
            return laser_170;
        case 260:
            return laser_260;
        case 340:
            return laser_340;
        case 481:
            return laser_481;
        case 551:
            return laser_511;
        default:
            return 0;
    }
}
void UpStairMode::legMA(int front,int back)
{			
    std_msgs::Int32MultiArray leg_MA;

    l_MA[0]=front;
    l_MA[1]=back;
    for (int i = 0; i < 2; i++)
    {
        leg_MA.data.push_back(l_MA[i]);
    }
    pub_lMA.publish(leg_MA);
    //	printf("leg_MA publish\n");

    ros::spinOnce();
}
void UpStairMode::robotMA(int left,int right)
{
    std_msgs::Int32MultiArray robot_MA;

    r_MA[0]=left;
    r_MA[1]=right;
    for (int i = 0; i < 2; i++)
    {
        robot_MA.data.push_back(r_MA[i]);
    }
    pub_rMA.publish(robot_MA);
    //	printf("robot_MA publish\n");

    ros::spinOnce();
}
void UpStairMode::robotspeed(int left,int right)
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
    //	printf("robot_speed publish\n");

    ros::spinOnce();
}
void UpStairMode::legspeed(int front,int back)
{
    std_msgs::Int32MultiArray leg_speed;

    l_speed[0]=front;
    l_speed[1]=back;
    for (int i = 0; i < 2; i++)
    {
        leg_speed.data.push_back(l_speed[i]);
    }
    pub_ls.publish(leg_speed);
    //	printf("legspeed publish\n");

    ros::spinOnce();
}

void UpStairMode::robotmotion(std::string rm)
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
    //printf("robot_motion publish\n");

    ros::spinOnce();
}
void UpStairMode::legmotion(std::string lm)
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
    //	printf("leg_motion publish\n");

    ros::spinOnce();
}
void UpStairMode::vw_control(float v, float w)
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
    UpStairMode::robotspeed(M2,M3);
    usleep(100000);
    UpStairMode::robotmotion("Forward");
    usleep(100000);
}
void UpStairMode::leghome(int front,int back)
{
    std_msgs::Int32MultiArray leg_HO;

    l_HO[0]=back;
    l_HO[1]=front;
    for (int i = 0; i < 2; i++)
    {
        leg_HO.data.push_back(l_HO[i]);
    }
    pub_lHO.publish(leg_HO);
    //	printf("leghome publish\n");

    ros::spinOnce();
}
void UpStairMode::motor_switch(int number,int serve_status)
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
void UpStairMode::wall_following_r(double rw_min,double rw_max,double vv,double ww,double Lfar_dis,double Lclose_dis,double Lstop_align_dis)	//---Alignment with laser & Calculated the tilt angle
{
    int i=0,j=0,x=0,y=0,mid_x=0,mid_y=0,sw,mid_x2=0,mid_y2=0;
    /*---For Alignment(Mode 22) Using PI Control---*/

    double sum_d23=0;
    int d=800;
    int err=15;
    //    double Lfar_dis=0.5,Lclose_dis=0.45,Lstop_align_dis=1;
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
    int time=0;

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

    ros::Rate r(2);

    UpStairMode::robotspeed(0,0);
    r.sleep();
    UpStairMode::robotmotion("Forward");
    r.sleep();

    int condition=0;
    while(laser_260>Lstop_align_dis &&ros::ok()&&!safe_switch)
    {
        ros::spinOnce();
        double Las_err=laser_32-laser_2;
        std::cout << "  laser32 = "<<laser_32<< std::endl;
        std::cout << "  laser2 = "<<laser_2<< std::endl;
        std::cout << "  Las_err = "<<Las_err<< std::endl;
        if(Las_err<rw_min )
        {
            if(laser_2>1.1) //if wall dis >0.3 and it is approach wall , don't care.
            {
                if(condition==3)
                    continue;
                condition=3;
                UpStairMode::vw_control(vv,0);//go straight
                sleep(1);
                printf("vv=%f,ww=0\n",vv);
            }
            else
            {
                if(condition==1)
                    continue;
                condition=1;
                UpStairMode::vw_control(vv,ww);//turn left
                sleep(1);
                printf("vv=%f,ww=%f\n",vv,ww);
            }

        }
        else if(Las_err>rw_max )
        {
            if(laser_2<0.5) //if wall dis <0.1 and it is depart from wall , don't care.
            {
                if(condition==3)
                    continue;
                condition=3;
                UpStairMode::vw_control(vv,0);//go straight
                sleep(1);
                printf("vv=%f,ww=0\n",vv);
            }
            else
            {
                if(condition==2)
                    continue;
                condition=2;
                UpStairMode::vw_control(vv,-ww);//turn right
                sleep(1);
                printf("vv=%f,ww=%f\n",vv,ww);
            }

        }
        else if(Las_err<0)
        {
            if(laser_2>1.1) //if wall dis >0.3 and it is approach wall , don't care.
            {
                if(condition==3)
                    continue;
                condition=3;
                UpStairMode::vw_control(vv,0);//go straight
                sleep(1);
                printf("vv=%f,ww=0\n",vv);
            }
            else
            {
                if(condition==1)
                    continue;
                condition=1;
                UpStairMode::vw_control(vv,ww);//turn left
                sleep(1);
                printf("vv=%f,ww=%f\n",vv,ww);
            }


        }
        else
        {
            if(laser_2>1.1)
            {
                if(condition==2)
                    continue;
                condition=2;
                UpStairMode::vw_control(vv,-ww);//turn right
                sleep(1);
                printf("vv=%f,ww=0\n",vv);
            }
            else if(laser_2 < 0.5)
            {
                if(condition==1)
                    continue;
                condition=1;
                UpStairMode::vw_control(vv,ww);//turn left
                sleep(1);
                printf("vv=%f,ww=%f\n",vv,ww);
            }
            else
            {
                if(condition==3)
                    continue;
                condition=3;
                UpStairMode::vw_control(vv,0);//go straight
                sleep(1);
                printf("vv=%f,ww=0\n",vv);
            }

        }
    }
    std::cout<<"Stop Alignment\n";
    int state=0;
    r.sleep();
    UpStairMode::robotspeed(350,350);
    r.sleep();
    while(ros::ok()&&!safe_switch){
        ros::spinOnce();
        if (laser_260>Lfar_dis){
            if(state==1)
                continue;
            state=1;
            UpStairMode::robotmotion("Forward");
            printf("laser_260=%f",laser_260);
            sleep(1);
        }
        else if(laser_260<Lclose_dis){
            if(state==2)
                continue;
            state=2;
            UpStairMode::robotmotion("Backward");
            printf("laser_260=%f",laser_260);
            sleep(1);
        }
        else
        {
            UpStairMode::robotmotion("Stop");;//stop
            sleep(1);
            std::cout<<"wall following end!\n";
            break;
        }

    }

    UpStairMode::robotmotion("Stop");
    r.sleep();
}
void UpStairMode::wall_following_l(double lw_min,double lw_max,double vv,double ww,double Lfar_dis,double Lclose_dis,double Lstop_align_dis)	//---Alignment with laser & Calculated the tilt angle
{
    int i=0,j=0,x=0,y=0,mid_x=0,mid_y=0,sw,mid_x2=0,mid_y2=0;
    /*---For Alignment(Mode 22) Using PI Control---*/

    double sum_d23=0;
    int err=15;
    //    double Lfar_dis=0.5,Lclose_dis=0.45,Lstop_align_dis=1;
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
    UpStairMode::robotspeed(0,0);
    r.sleep();
    UpStairMode::robotmotion("Forward");
    r.sleep();
    int condition=0;
    while(laser_260>Lstop_align_dis &&ros::ok()&&!safe_switch)
    {
        ros::spinOnce();
        double Las_err=laser_481-laser_511;
        std::cout << "  laser481 = "<<laser_481<< std::endl;
        std::cout << "  laser511 = "<<laser_511<< std::endl;
        std::cout << "  Las_err = "<<Las_err<< std::endl;
        if(Las_err<lw_min )
        {
            if(laser_511>1)
            {
                if(condition==3)
                    continue;
                condition=3;
                UpStairMode::vw_control(vv,0);//go straight
                usleep(100000);
                printf("vv=%f,ww=0\n",vv);
            }
            else
            {
                if(condition==2)
                    continue;
                condition=2;
                UpStairMode::vw_control(vv,-ww);//turn right
                usleep(100000);
                printf("vv=%f,ww=%f\n",vv,-ww);
            }

        }
        else if(Las_err>lw_max)
        {
            if(laser_511<0.5)
            {
                if(condition==3)
                    continue;
                condition=3;
                UpStairMode::vw_control(vv,0);//go straight
                usleep(100000);
                printf("vv=%f,ww=0\n",vv);
            }
            else{
                if(condition==1)
                    continue;
                condition=1;
                UpStairMode::vw_control(vv,ww);//turn left
                usleep(100000);
                printf("vv=%f,ww=%f\n",vv,-ww);
            }

        }
        else if(Las_err<0)
        {
            if(laser_511>1)
            {
                if(condition==3)
                    continue;
                condition=3;
                UpStairMode::vw_control(vv,0);//go straight
                usleep(100000);
                printf("vv=%f,ww=0\n",vv);
            }
            else
            {
                if(condition==2)
                    continue;
                condition=2;
                UpStairMode::vw_control(vv,-ww);//turn right
                usleep(100000);
                printf("vv=%f,ww=%f\n",vv,-ww);
            }
        }
        else
        {
            if(laser_511>1)
            {
                if(condition==1)
                    continue;
                condition=1;
                UpStairMode::vw_control(vv,ww);//turn left
                usleep(100000);
            }
            else if(laser_511<0.5)
            {
                if(condition==2)
                    continue;
                condition=2;
                UpStairMode::vw_control(vv,-ww);//turn right
                usleep(100000);
            }
            else
            {
                if(condition==3)
                    continue;
                condition=3;
                UpStairMode::vw_control(vv,0);//go straight
                usleep(100000);
                printf("vv=%f,ww=0\n",vv);
            }


        }

    }
    std::cout<<"Stop Alignment\n";

    r.sleep();
    UpStairMode::robotspeed(350,350);
    r.sleep();
    int state=0;

    while(ros::ok()&&!safe_switch){
        ros::spinOnce();
        if (laser[260]>Lfar_dis){
            if(state==1)
                continue;
            state=1;
            UpStairMode::robotmotion("Forward");
            printf("laser_260=%f",laser[260]);
            usleep(100000);
        }
        else if(laser[260]<Lclose_dis){
            if(state==2)
                continue;
            state=2;
            UpStairMode::robotmotion("Backward");
            printf("laser_260=%f",laser[260]);
            usleep(100000);
        }
        else
        {
            UpStairMode::robotmotion("Stop");;//stop
            usleep(100000);
            std::cout<<"wall following end!\n";
            break;
        }

    }
    UpStairMode::robotmotion("Stop");
    r.sleep();
}
void UpStairMode::wall_alignment_r(double rw_min,double rw_max)	//---Alignment with laser & Calculated the tilt angle
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
    int condition=0;
    ros::Rate r(2);

    UpStairMode::robotspeed(0,0);
    r.sleep();
    UpStairMode::robotmotion("Forward");
    r.sleep();

    printf("Start Angle aligment!\n");
    while(ros::ok()&&!safe_switch)
    {
        ros::spinOnce();
        double Las_err=laser[48]-laser[16];
        if(laser[48]>5)
            laser[48]=5;
        if(laser[16]>5)
            laser[16]=5;
        if(laser[16]==5&&laser[48]==5)
            Las_err=0.5;

        std::cout << "  laser48 = "<<laser[48]<< std::endl;
        std::cout << "  laser16 = "<<laser[16]<< std::endl;
        std::cout << "  Las_err = "<<Las_err<< std::endl;
        if(Las_err<rw_min)
        {
            if(condition==1)
                continue;
            condition=1;
            UpStairMode::vw_control(0,ww);//turn left
            usleep(100000);
            printf("vv=%f,ww=%f\n",vv,ww);

        }
        else if(Las_err>rw_max)
        {
            if(condition==2)
                continue;
            condition=2;
            UpStairMode::vw_control(0,-ww);//turn right
            usleep(100000);
            printf("vv=%f,ww=%f\n",vv,ww);

        }
        else if(Las_err<0)
        {
            if(condition==1)
                continue;
            condition=1;
            UpStairMode::vw_control(0,ww);//turn left
            usleep(100000);
            printf("vv=%f,ww=%f\n",vv,ww);

        }
        else
        {
            UpStairMode::robotmotion("Stop");//Stop
            usleep(100000);
            break;
        }
    }
    std::cout<<"Stop Alignment\n";





//    r.sleep();


    UpStairMode::robotmotion("Stop");
    r.sleep();
}
void UpStairMode::wall_alignment_l(double rw_min,double rw_max)	//---Alignment with laser & Calculated the tilt angle
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
    int condition =0;
    ros::Rate r(2);

    UpStairMode::robotspeed(0,0);
    r.sleep();
    UpStairMode::robotmotion("Forward");
    r.sleep();

    printf("Start Angle aligment!\n");
    while(ros::ok()&&!safe_switch)
    {
        ros::spinOnce();
//        double Las_err=laser_481-laser_511;
        double Las_err=laser[463]-laser[495];
        std::cout << "  laser_481 = "<<laser_481<< std::endl;
        std::cout << "  laser_511 = "<<laser_511<< std::endl;
        std::cout << "  Las_err = "<<Las_err<< std::endl;
        if(Las_err<rw_min)
        {
            if(condition==2)
                continue;
            condition=2;
            UpStairMode::vw_control(0,-ww);//turn right
            usleep(100000);
            printf("vv=%f,ww=%f\n",vv,-ww);

        }
        else if(Las_err>rw_max)
        {
            if(condition==1)
                continue;
            condition=1;
            UpStairMode::vw_control(0,ww);//turn left
            usleep(100000);
            printf("vv=%f,ww=%f\n",vv,ww);

        }
        else if(Las_err<0)
        {
            if(condition==2)
                continue;
            condition=2;
            UpStairMode::vw_control(0,-ww);//turn right
            usleep(100000);
            printf("vv=%f,ww=%f\n",vv,-ww);

        }
        else
        {
            UpStairMode::robotmotion("Stop");;//Stop
            usleep(100000);
            break;
        }
    }
    std::cout<<"Stop Alignment\n";





//    r.sleep();


    UpStairMode::robotmotion("Stop");
    r.sleep();
}
void UpStairMode::wall_approach_laser(double vv,double ww,double Lfar_dis,double Lclose_dis,double Lstop_align_dis)	//---Alignment with laser & Calculated the tilt angle
{
    int i=0,j=0,x=0,y=0,mid_x=0,mid_y=0,sw,mid_x2=0,mid_y2=0;
    /*---For Alignment(Mode 22) Using PI Control---*/

    double sum_d23=0;
    int err=15;
    //    double Lfar_dis=0.5,Lclose_dis=0.45,Lstop_align_dis=1;
    float Kp=1.0,Ki=0.1;
    ros::param::get(NameSpace+"/Kp", Kp);
    ros::param::get(NameSpace+"/Ki", Ki);
    ros::param::get(NameSpace+"/err", err);
    //    ros::param::get(NameSpace+"/vv", vv);

    //    std::cout<<"err="<<err<<"\n";
    //    std::cout<<"stop_align_dis="<<stop_align_dis<<"\n";

    /*---Vel - For Record---*/
    int vel_L=0,vel_R=0;
    int cnt=0,condition=0;


    printf("wall_approach_laser start ...\n");
    //=======ROS============

    ros::Rate r(2);

    UpStairMode::robotmotion("Forward");
    r.sleep();
    if(laser_260>2.5)
        laser_260=2.5;
    while(laser_260>Lstop_align_dis &&ros::ok()&&!safe_switch)
    {
        ros::spinOnce();
        if(laser_340>2.5)
            laser_340=2.5;
        if(laser_170>2.5)
            laser_170=2.5;
        if(laser_260>2.5)
            laser_260=2.5;

        double Las_err=laser_340-laser_170;
//        std::cout << "  laser_340 = "<<laser_340<< std::endl;
//        std::cout << "  laser_170 = "<<laser_170<< std::endl;
        std::cout << "  Las_err = "<<Las_err<< std::endl;
        if(Las_err>0.1)
        {
            if(condition==1)
                continue;
            condition=1;
            UpStairMode::vw_control(vv,-ww);
        }
        else if(Las_err<-0.1)
        {
            if(condition==2)
                continue;
            condition=2;
            UpStairMode::vw_control(vv,ww);
        }
        else
        {
            UpStairMode::vw_control(vv,0);
        }
        usleep(100000);
    }
    std::cout<<"Stop Alignment\n";
    int cc=0;
    r.sleep();
    UpStairMode::robotspeed(350,350);
    r.sleep();
    while(ros::ok()&&!safe_switch){

        ros::spinOnce();
        if (laser_260>Lfar_dis){
            if(cc==1)
                continue;
            cc=1;
            UpStairMode::robotmotion("Forward");
            printf("laser_260=%f",laser_260);
            usleep(100000);
        }
        else if(laser_260<Lclose_dis){
            if(cc==2)
                continue;
            cc=2;
            UpStairMode::robotmotion("Backward");
            printf("laser_260=%f",laser_260);
            usleep(100000);
        }
        else
        {
            UpStairMode::robotmotion("Stop");;//stop
            usleep(100000);
            std::cout<<"wall following end!\n";
            break;
        }

    }
    UpStairMode::robotmotion("Stop");
    r.sleep();
}
