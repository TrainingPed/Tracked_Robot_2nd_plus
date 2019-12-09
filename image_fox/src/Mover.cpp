//using fuzzy control  
//ARRAYSIZE =1000; over will lead to segmentation fault
#include "Mover.h"

using namespace std;
void robot::savetotxt(std::string dir)
{
//    dir="/home/nvidia/move_data/";
    if (access(dir.c_str(), 0) == -1)
    {
        cout<<dir<<" is not existing"<<endl;
        cout<<"now make it"<<endl;

        int flag=mkdir(dir.c_str(), 0777);

        if (flag == 0)
            cout<<"make successfully"<<endl;
        else
            cout<<"make errorly"<<endl;
    }

    if (access(dir.c_str(), 0) == 0)
        cout<<dir<<" exists"<<endl;

    cout<<"end..."<<endl;

    //std::cout<<"Please enter filename ...\n";
    //std::getline(std::cin, filename);
//    if(filename =="n"||filename =="")
//    {
        std::cout<<"filename save as time...\n";
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);

        std::ostringstream oss;
        oss << std::put_time(&tm, "%m%d-%H%M");
        filename = oss.str();
//    }

    filename=dir+filename;//save as txt
    nh.setParam("filename",filename);
    std::cout<<" filename = "<<filename<<" \n";
}
void robot::robotmotion(std::string rm)
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

    rm_pub.publish(robot_motion);
    //printf("robot_motion publish\n");

    ros::spinOnce();
}
void robot::robotspeed(int left,int right)
{
    std_msgs::Int32MultiArray robot_speed;
    robot_speed.data.push_back(left);
    robot_speed.data.push_back(right);
    rs_pub.publish(robot_speed);
    //	printf("robot_speed publish\n");

    ros::spinOnce();
}
void robot::Banlist(vector<double> x, vector<double> y, vector<double> r,std::string name)
{
    XmlRpc::XmlRpcValue Xml_x,Xml_y,Xml_r;
    Xml_x.setSize(x.size());
    Xml_y.setSize(y.size());
    Xml_r.setSize(r.size());
    for(int i=0;i<x.size();i++)
    {
        Xml_x[i]=x[i];
        Xml_y[i]=y[i];
        Xml_r[i]=r[i];
    }
    std::string nx=name+"_ban_x";
    std::string ny=name+"_ban_y";
    std::string nr=name+"_ban_r";
    nh.setParam(nx,Xml_x);
    nh.setParam(ny,Xml_y);
    nh.setParam(nr,Xml_r);
}
bool robot::Banrange(string path_name_x)
{
    vector<double> ban_x;
    vector<double> ban_y;
    vector<double> ban_r;
    nh.getParam(path_name_x+"_ban_x", ban_x);
    nh.getParam(path_name_x+"_ban_y", ban_y);
    nh.getParam(path_name_x+"_ban_r", ban_r);
    for (int i=0;i<ban_x.size();i++)
    {
        if(pow((a.x -ban_x[i]),2.0)+pow((a.y-ban_y[i]),2.0)<pow(ban_r[i],2.0))//if robot was at the ban area ,then return true.
            return true;
    }
    return false;
}
void robot::MotorSwitch(int motor_number, int servostatus)
{
        std_msgs::Int32MultiArray motorswitch;
        motorswitch.data.push_back(motor_number);
        motorswitch.data.push_back(servostatus);
        motor_pub.publish(motorswitch);
        ros::spinOnce();
}
void robot::Posepub(double px, double py, double pz, double ox, double oy, double oz, double ow, double theta,bool odom_caliberate)
{
    std_msgs::Float32MultiArray pose,odom;
    if(odom_caliberate)
    {
        odom.data.push_back(px);
        odom.data.push_back(py);
        odom.data.push_back(theta);
        odom_set.publish(odom);//publish pose to odometry
        ros::spinOnce();
    }

    pose.data.push_back(px);
    pose.data.push_back(py);
    pose.data.push_back(pz);//pz=0
    pose.data.push_back(ox);
    pose.data.push_back(oy);
    pose.data.push_back(oz);
    pose.data.push_back(ow);
    pose.data.push_back(theta);
    pose_pub.publish(pose);//publish pose to amcl

    xm=px;//update modified odom
    ym=py;//update modified odom
    thm=theta;//update modified odom
    o.x=px;
    o.y=py;
    o.th=theta;
    ros::spinOnce();
    ROS_INFO("Set pose : %.3f %.3f %.3f", px, py, theta);
}
void robot::vw_control(float v, float w)
{
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
    robot::robotspeed(M2,M3);
    usleep(5000);
    robot::robotmotion("Forward");
    usleep(5000);
}
double robot::max(double test)
{
	double temp;
	if (test > 0)
		temp = test;
	else
		temp = 0;
	return(temp);
}
void robot::send_position(double x, double y, string Floor, string status  )
{



	if (x == 9999)
	{
		x = old_x;
	}
	if (y == 9999)
	{
		y = old_y;
	}
	if (Floor == "moving")
	{
		Floor = old_Floor;
	}
	//if (status == "none")
	//{
	//	status = old_status;
	//}
	
	//cout<< x << "," << y << "," << Floor << "," << status << std::endl;
    nh.getParam("filename",filename);
	string position_file=filename+"position.txt";
	string info_file=filename+"info.txt";
	ofstream infofile(info_file.c_str());
	infofile<< x << "," << y << "," << Floor << "," << status << std::endl;
	ofstream positionfile(position_file.c_str(), std::ios::app);
    positionfile << x << "," << y << std::endl;
	//cout << "printing info and status!"<<endl;
	old_y = y;
	old_x = x;
	//old_status = status;
	old_Floor = Floor;
}
void robot::send_robotpose(double x, double y)
{
    std_msgs::Float32MultiArray array;
    array.data.push_back(x);
    array.data.push_back(y);
    rp_pub.publish(array);
    ros::spinOnce();
}
void robot::lasercallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{

	laser2 = msg->ranges[2];
	laser340 = msg->ranges[340];
	laser360 = msg->ranges[360];
	laser380 = msg->ranges[380];
	laser718 = msg->ranges[718];
    for(int i=0;i<512;i++){
        laser[i]=msg->ranges[i];
    }


}

void robot::SWcallback(const std_msgs::Int32::ConstPtr &msg)
{
	SW = msg->data;
	std::cout << "sub SW = " << SW << "\n";

}

void robot::sonarcallback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
	for (int i = 0; i < 8; i++)
	{
		sonarx[i] = msg->points[i].x;
		sonary[i] = msg->points[i].y;
		//sonardist = (sonarx[3] + sonarx[4]) / 2;
		sonar0 = sonary[0];
		sonar7 = sonary[7];
		//std::cout << " sonar dist: " << sonardist << std::endl;
		/*std::cout << "  data x" << i << ": " << sonarx[i] << " data y" << i << ": " << sonary[i] << std::endl;*/
	}
	std::cout << std::endl;
}

void robot::odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{

    o.x = msg->pose.pose.position.x;
    o.y = msg->pose.pose.position.y;
	tf::Pose pose;
	tf::poseMsgToTF(msg->pose.pose, pose);
    o.th = tf::getYaw(pose.getRotation());
    o.ox=msg->pose.pose.orientation.x;
    o.oy=msg->pose.pose.orientation.y;
    o.oz=msg->pose.pose.orientation.z;
    o.ow=msg->pose.pose.orientation.w;



    //std::cout << "Update Odometry :";
    //std::cout << " x =" << x_current << " y =" << y_current << std::endl;//<< " th = " << th_p<<" "

}
void robot::amcl_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{

	a.x = msg->pose.pose.position.x;
	a.y = msg->pose.pose.position.y;
	tf::Pose pose;
	tf::poseMsgToTF(msg->pose.pose, pose);
	a.th = tf::getYaw(pose.getRotation());
    a.ox=msg->pose.pose.orientation.x;
    a.oy=msg->pose.pose.orientation.y;
    a.oz=msg->pose.pose.orientation.z;
    a.ow=msg->pose.pose.orientation.w;
    a.covx=msg->pose.covariance[0];
    a.covy=msg->pose.covariance[7];
    a.covth=msg->pose.covariance[35];

}

void robot::vo_callback(const nav_msgs::Odometry::ConstPtr& msg)
{

	vo.x = msg->pose.pose.position.x;
	vo.y = msg->pose.pose.position.y;
	tf::Pose pose;
	tf::poseMsgToTF(msg->pose.pose, pose);
	vo.th = tf::getYaw(pose.getRotation());
}

void robot::astarCallback(const nav_msgs::Path& msg)//Recive A* path
{
	path = msg;
	if (msg.poses.empty())
	{
		printf("Path is empty!\n");
	}
	for (int j = 0; j < path.poses.size(); j++)//Recive A* path
	{
		xr[j] = (path.poses[j].pose.position.x);
		yr[j] = (path.poses[j].pose.position.y);
		//printf("Header: %s. Id: %d. (%f, %f)", path.header.frame_id, path.header.stamp, path.poses[i].pose.position.x, path.poses[i].pose.position.y);
		//std::cout << " xr= " << path.poses[j].pose.position.x << " yr=" << path.poses[j].pose.position.y << std::endl;
	}

}

//void robot::a_star_callback_x(const std_msgs::Float64MultiArray::ConstPtr& array)
//{
//	for (int i = 0; i<ARRAYSIZE; i++)
//	{
//		xr[i] = *(array->data.begin() + i);
//	}
//}
//void robot::a_star_callback_y(const std_msgs::Float64MultiArray::ConstPtr& array)
//{
//	for (int i = 0; i<ARRAYSIZE; i++)
//	{
//		yr[i] = *(array->data.begin() + i);
//	}
//}
void robot::resetodom()//Reset odometry
{
	std_msgs::Int32 reset;
	reset.data = 1;
	resetodom_pub.publish(reset);
	robot::pause_time(1);
}
double robot::coordinate_transform(int mode, double x, double y, double th)
{
	double xt = x;
	double yt = y;
	double tht = th;
	if (mode == 1)//set x y to zero. no change direction.
	{
		x = x - xt;
		y = y - yt;
		th = th - tht;
	}
	if (mode == 2)//change direction to 180 degree reverse. 
	{
		x = -x + xt;
		y = -y + yt;
		th = th - tht;
	}
	if (mode == 3)//change direction to 90 degree (counterclockwise)
	{
		x = yt - y;
		y = x - xt;
		th = th - tht;
	}
	if (mode == 4)//change direction to 270 degree (counterclockwise)
	{
		x = y - yt;
		y = x - xt;
		th = th - tht;
	}
	return x, y, th;
}

velocity robot::kinematic(int i, double x, double y, double x1, double y1,int amcl_modify,string path_name_x)
{
    double path_follow_range=0.2;
    int odom_belief_points=5;
    int odom_reset_points=15;
    nh.getParam("odom_belief_points",odom_belief_points);
    nh.getParam("odom_reset_points",odom_reset_points);
	nh.getParam("obstacle_avoid",obstacle);
    nh.getParam("path_follow_range",path_follow_range);


    robot::fusion(i,amcl_modify,path_name_x);
    if(obstacle)
	robot::obstacle_avoid();
	
	thr[i + 1] = atan2((y1 - y), (x1 - x));
	thr[0] = thr[1];//modify the first angle.
	//double thrr[ARRAYSIZE] = { 0 };
	//thrr[i]=thr[i + 1] - thr[i];

	int j = 0, k = 0;
	double premv[5][5][5];   /* matrix holds premise of all 125 mem. funct. */
	double premw[5][5][5];   /* matrix holds premise of all 125 mem. funct. */
	double mfex[5];                 /* array holds value of membership funct. of e(t) */
	double mfey[5];              /* array holds value of membership funct. of edot(t) */
	double mfeth[5];
	double tablev[5][5][5] =   /* fills rule table #1*/
    { { { -0.2,-0.2,-0.2,-0.2,-0.2 },{ -0.2,-0.2,-0.2,-0.2,-0.2 },{ -0.2,-0.2,-0.2,-0.2,-0.2 },{ -0.2,-0.2,-0.2,-0.2,-0.2 },{ -0.2,-0.2,-0.2,-0.2,-0.2 } },
    { { -0.2,-0.2,-0.2,-0.2,-0.2 },{ -0.1,-0.1,-0.1,-0.1,-0.1 },{ -0.1,-0.1,-0.1,-0.1,-0.1 },{ -0.1,-0.1,-0.1,-0.1,-0.1 },{ -0.2,-0.2,-0.2,-0.2,-0.2 } },
    { { -0.2,-0.2,-0.2,-0.2,-0.2 },{ -0.1,-0.1,-0.1,-0.1,-0.1 },{ 0.0,0.0,0.0,0.0,0.0 },{ -0.1,-0.1,-0.1,-0.1,-0.1 },{ -0.2,-0.2,-0.2,-0.2,-0.2 } },
    { { 0.2,0.2,0.2,0.2,0.2 },{ 0.1,0.1,0.1,0.1,0.1 },{ 0.1,0.1,0.1,0.1,0.1 },{ 0.1,0.1,0.1,0.1,0.1 },{ 0.2,0.2,0.2,0.2,0.2 } },
    { { 0.2,0.2,0.2,0.2,0.2 },{ 0.2,0.2,0.2,0.2,0.2 },{ 0.2,0.2,0.2,0.2,0.2 },{ 0.2,0.2,0.2,0.2,0.2 },{ 0.2,0.2,0.2,0.2,0.2 } },
    };
//	{{{-0.30,-0.30,-0.30,-0.30,-0.30},{-0.30,-0.30,-0.30,-0.30,-0.30},{-0.30,-0.30,-0.30,-0.30,-0.30},{-0.30,-0.30,-0.30,-0.30,-0.30},{-0.30,-0.30,-0.30,-0.30,-0.30}},
//	{{-0.30,-0.30,-0.30,-0.30,-0.30},{-0.15,-0.15,-0.15,-0.15,-0.15},{-0.15,-0.15,-0.15,-0.15,-0.15},{-0.15,-0.15,-0.15,-0.15,-0.15},{-0.30,-0.30,-0.30,-0.30,-0.30}},
//	{{-0.30,-0.30,-0.30,-0.30,-0.30},{-0.15,-0.15,-0.15,-0.15,-0.15},{0.00,0.00,0.00,0.00,0.00},{-0.15,-0.15,-0.15,-0.15,-0.15},{-0.30,-0.30,-0.30,-0.30,-0.30}},
//	{{0.30,0.30,0.30,0.30,0.30},{0.15,0.15,0.15,0.15,0.15},{0.15,0.15,0.15,0.15,0.15},{0.15,0.15,0.15,0.15,0.15},{0.30,0.30,0.30,0.30,0.30}},
//	{{0.30,0.30,0.30,0.30,0.30},{0.30,0.30,0.30,0.30,0.30},{0.30,0.30,0.30,0.30,0.30},{0.30,0.30,0.30,0.30,0.30},{0.30,0.30,0.30,0.30,0.30}},
//	};
	//{{{-0.40,-0.40,-0.40,-0.40,-0.40},{-0.40,-0.40,-0.40,-0.40,-0.40},{-0.40,-0.40,-0.40,-0.40,-0.40},{-0.40,-0.40,-0.40,-0.40,-0.40},{-0.40,-0.40,-0.40,-0.40,-0.40}},
	//{{-0.40,-0.40,-0.40,-0.40,-0.40},{-0.20,-0.20,-0.20,-0.20,-0.20},{-0.20,-0.20,-0.20,-0.20,-0.20},{-0.20,-0.20,-0.20,-0.20,-0.20},{-0.40,-0.40,-0.40,-0.40,-0.40}},
	//{{-0.40,-0.40,-0.40,-0.40,-0.40},{-0.20,-0.20,-0.20,-0.20,-0.20},{0.00,0.00,0.00,0.00,0.00},{-0.20,-0.20,-0.20,-0.20,-0.20},{-0.40,-0.40,-0.40,-0.40,-0.40}},
	//{{0.40,0.40,0.40,0.40,0.40},{0.20,0.20,0.20,0.20,0.20},{0.20,0.20,0.20,0.20,0.20},{0.20,0.20,0.20,0.20,0.20},{0.40,0.40,0.40,0.40,0.40}},
	//{{0.40,0.40,0.40,0.40,0.40},{0.40,0.40,0.40,0.40,0.40},{0.40,0.40,0.40,0.40,0.40},{0.40,0.40,0.40,0.40,0.40},{0.40,0.40,0.40,0.40,0.40}},
	//};
			
	//-0.4, -0.2, 0.1, 0.2, 0.4,
	//-0.3, -0.1, 0, 0.1, 0.3,
	//-0.2, -0.1, 0, 0.1, 0.2,
	//-0.3, -0.1, 0, 0.1, 0.3,
	//-0.4, -0.2, 0.1,0.2, 0.4		

	double tablew[5][5][5] =  /* fills rule table #2*/
//	{ { { 0.00,0.24,0.48,0.48,0.48 },{ -0.24,0.00,0.24,0.48,0.48 },{ -0.48,-0.24,0.00,0.24,0.48 },{ -0.48,-0.48,-0.24,0.00,0.24 },{ 0.48,-0.48,-0.48,-0.24,0.00 } },
//	{ { 0.24,0.48,0.48,0.48,-0.48 },{ 0.00,0.24,0.48,0.48,0.48 },{ -0.48,-0.24,0.00,0.24,0.48 },{ 0.48,-0.48,-0.48,-0.24,0.00 },{ 0.48,-0.48,-0.48,-0.48,-0.24 } },
//	{ { 0.48,0.48,0.48,-0.48,-0.48 },{ 0.48,0.48,0.48,-0.48,-0.48 },{ 0.48, 0.24, 0,-0.24,-0.48 },{ 0.48,0.48,0.48,-0.48,-0.48 },{ 0.48,0.48,0.48,-0.48,-0.48 } },
//	{ { 0.48,-0.48,-0.48,-0.48,-0.24 },{ -0.48,-0.48,-0.48,-0.24,0.00 },{ -0.48,-0.24,0.00,0.24,0.48 },{ 0.00,0.24,0.48,0.48,0.48 },{ 0.24,0.48,0.48,0.48,-0.48 } },
//	{ { -0.48,-0.48,-0.48,-0.24,0.00 },{ -0.48,-0.48,-0.24,0.00,0.24 },{ -0.48,-0.24,0.00,0.24,0.48 },{ -0.24,0.00,0.24,0.48,0.48 },{ 0.00,0.24,0.48,0.48,0.48 } },
//	};
	//{ { { 0.00,0.00,0.24,0.24,0.24 },{ 0.00,0.00,0.00,0.00,0.24 },{ -0.24,-0.24,0.00,0.00,0.00 },{ -0.12,-0.24,-0.24,-0.24,0.00 },{ -0.12,-0.12,-0.12,-0.24,-0.24 } },
	//{ { 0.24,0.24,0.24,0.24,0.12 },{ 0.00,0.00,0.24,0.24,0.24 },{ -0.24,-0.24,0.00,0.00,0.00 },{ -0.12,-0.12,-0.12,-0.24,-0.24 },{ -0.24,-0.12,-0.12,-0.12,-0.12 } },
	//{ { 0.24,0.24,-0.24,-0.12,-0.12 },{ 0.24,0.24,-0.24,-0.12,-0.12 },{ 0.24,0.12,0,-0.12,-0.24 },{ 0.24,0.24,0.12,-0.12,-0.12 },{ 0.24,0.24,0.12,-0.12,-0.12 } },
	//{ { -0.24,-0.12,-0.12,-0.12,-0.12 },{ -0.12,-0.12,-0.12,-0.24,-0.24 },{ -0.24,-0.24,0.00,0.00,0.00 },{ 0.00,0.00,0.24,0.24,0.24 },{ 0.24,0.24,0.24,0.24,0.12 } },
	//{ { -0.12,-0.12,-0.12,-0.24,-0.24 },{ -0.12,-0.24,-0.24,-0.24,0.00 },{ -0.24,-0.24,0.00,0.00,0.00 },{ 0.00,0.00,0.00,0.00,0.24 },{ 0.00,0.00,0.24,0.24,0.24 } },
	//};

//    { { { 0.00,0.09,0.18,0.18,0.18 },{ -0.09,0.00,0.09,0.18,0.18 },{ -0.18,-0.09,0.00,0.09,0.18 },{ -0.18,-0.18,-0.09,-0.00,0.09 },{ -0.18,-0.18,-0.18,-0.09,-0.00 } },
//        { { 0.09,0.18,0.18,0.18,-0.18 },{ 0.00,0.09,0.18,0.18,0.18 },{ -0.18,-0.09,0.00,0.09,0.18 },{ 0.18,-0.18,-0.18,-0.09,-0.00 },{ -0.18,-0.18,-0.18,-0.18,-0.09 } },
//        { { 0.18,0.18,-0.18,-0.18,-0.18 },{ 0.18,0.18,-0.18,-0.18,-0.18 },{ 0.18,0.09,0,-0.09,-0.18 },{ 0.18,0.18,0.18,-0.18,-0.18 },{ 0.18,0.18,0.18,-0.18,-0.18 } },
//        { { -0.18,-0.18,-0.18,-0.18,-0.09 },{ -0.18,-0.18,-0.18,-0.09,-0.00 },{ -0.18,-0.09,0.00,0.09,0.18 },{ 0.00,0.09,0.18,0.18,0.18 },{ 0.09,0.18,0.18,0.18,-0.18 } },
//        { { -0.18,-0.18,-0.18,-0.09,-0.00 },{ -0.18,-0.18,-0.09,-0.00,0.09 },{ -0.18,-0.09,0.00,0.09,0.18 },{ -0.09,0.00,0.09,0.18,0.18 },{ 0.00,0.09,0.18,0.18,0.18 } },
//    };

                { { { 0.00,0.06,0.12,0.12,0.12 },{ -0.06,0.00,0.06,0.12,0.12 },{ -0.12,-0.06,0.00,0.06,0.12 },{ -0.12,-0.12,-0.06,-0.00,0.06 },{ -0.12,-0.12,-0.12,-0.06,-0.00 } },
        { { 0.06,0.12,0.12,0.12,-0.12 },{ 0.00,0.06,0.12,0.12,0.12 },{ -0.12,-0.06,0.00,0.06,0.12 },{ 0.12,-0.12,-0.12,-0.06,-0.00 },{ -0.12,-0.12,-0.12,-0.12,-0.06 } },
        { { 0.12,0.12,-0.12,-0.12,-0.12 },{ 0.12,0.12,-0.12,-0.12,-0.12 },{ 0.12,0.06,0,-0.06,-0.12 },{ 0.12,0.12,0.12,-0.12,-0.12 },{ 0.12,0.12,0.12,-0.12,-0.12 } },
        { { -0.12,-0.12,-0.12,-0.12,-0.06 },{ -0.12,-0.12,-0.12,-0.06,-0.00 },{ -0.12,-0.06,0.00,0.06,0.12 },{ 0.00,0.06,0.12,0.12,0.12 },{ 0.06,0.12,0.12,0.12,-0.12 } },
        { { -0.12,-0.12,-0.12,-0.06,-0.00 },{ -0.12,-0.12,-0.06,-0.00,0.06 },{ -0.12,-0.06,0.00,0.06,0.12 },{ -0.06,0.00,0.06,0.12,0.12 },{ 0.00,0.06,0.12,0.12,0.12 } },
    };




	/*for (k = 0; k < 25; k++)
	{
	std::cout << " tablew[0][0][k] = " << tablew[0][0][k] << std::endl;
	}*/
	//	0.1, 0.05, 0, -0.05, -0.1,
	//0.2, 0.1, 0, -0.1, -0.2,
	//-0.3, -0.15, 0, 0.15, 0.3,
	//-0.2,-0.1, 0, 0.1, 0.2,	
	//-0.1, -0.05, 0, 0.05, 0.1

	double centex[5] = { -0.2,-0.1,0,0.1,0.2 };                  /* fills array with centers of ex membership functions*/
	double centey[5] = { -0.2,-0.1,0,0.1,0.2 };                  /* fills array with centers of ey membership functions*/
																 //double centeth[5]={-0.785,-0.393,0,0.393,0.785};                  /* fills array with centers of eth membership functions*/
	double centeth[5] = { -0.785 / 2,-0.393 / 2,0,0.393 / 2,0.785 / 2 };                  /* fills array with centers of eth membership functions*/

	double leftex[5] = { 0.2,0.15,0.1,0.1,0.1 };    /* defines the points at which the left side of the base of each triangle on the e universe of discourse hits the horozontal axis */
	double leftey[5] = { 0.2,0.15,0.1,0.1,0.1 };    /* defines the points at which the left side of the base of each triangle on the e universe of discourse hits the horozontal axis */
													//double lefteth[5]={0.785,0.588,0.393,0.393,0.393};    /* defines the points at which the left side of the base of each triangle on the e universe of discourse hits the horozontal axis */      
	double lefteth[5] = { 0.785 / 2,0.588 / 2,0.393 / 2,0.393 / 2,0.393 / 2 };

	double rightex[5] = { 0.1,0.1,0.1,0.15,0.2 }; /* defines the points at which the right side of the base of each triangle on the e universe of discourse hits the horozontal axis */
	double rightey[5] = { 0.1,0.1,0.1,0.15,0.2 }; /* defines the points at which the right side of the base of each triangle on the e universe of discourse hits the horozontal axis */
												  //double righteth[5]={0.393,0.393,0.393,0.588,0.785}; /* defines the points at which the right side of the base of each triangle on the e universe of discourse hits the horozontal axis */
	double righteth[5] = { 0.393 / 2,0.393 / 2,0.393 / 2,0.588 / 2,0.785 / 2 };

	double ex = 0, ey = 0, testx = 0, testy = 0, eth = 0, testth = 0;
	//ex = x - o.x;
	//ey = y - o.y;
	ex = (x - x_current)*cos(thr[i])+(y - y_current)*sin(thr[i]);
	ey = -(x - x_current)*sin(thr[i])+(y - y_current)*cos(thr[i]);
	eth = thr[i] - theta_current;
    if (abs(thr[i+1]-thr[i]) > 0.1||abs(thr[i]-thr[i-1]) > 0.1||abs(thr[i-1]-thr[i-2]) > 0.1)
    {
        if(ex<0.05 )
            path_follow=false;
        else
            path_follow=true;
    }
//    else if(ey>0.15)
//    {
//        if(ex<0.05)
//            path_follow=false;
//        else
//            path_follow=true;
//    }
    else
    {
        if(ex<path_follow_range )
            path_follow=false;
        else
            path_follow=true;
    }
	if (eth > 3.14)//if eth >180 then -360 degree
	{
		eth = eth - 3.14 * 2;
	}
	if (eth < -3.14)//if eth < -180 then +360 degree
	{
		eth = eth + 3.14 * 2;
	}

	if (ex >= 0.2)
		mfex[4] = 1, mfex[0] = 0, mfex[1] = 0, mfex[2] = 0, mfex[3] = 0;
	else if (ex <= -0.2)
		mfex[0] = 1, mfex[1] = 0, mfex[2] = 0, mfex[3] = 0, mfex[4] = 0;
	else {
		for (k = 0; k <= 4; k++) {
			if (ex < centex[k]) {
				testx = 1 - (centex[k] - ex) / leftex[k];
				mfex[k] = robot::max(testx);
			}
			else {
				testx = 1 - (ex - centex[k]) / rightex[k];
				mfex[k] = robot::max(testx);
			}
		}
	}

	if (ey >= 0.2)
		mfey[4] = 1, mfey[0] = 0, mfey[1] = 0, mfey[2] = 0, mfey[3] = 0;
	else if (ey <= -0.2)
		mfey[0] = 1, mfey[1] = 0, mfey[2] = 0, mfey[3] = 0, mfey[4] = 0;
	else {
		for (k = 0; k <= 4; k++) {
			if (ey < centey[k]) {
				testy = 1 - (centey[k] - ey) / leftey[k];
				mfey[k] = robot::max(testy);
			}
			else {
				testy = 1 - (ey - centey[k]) / rightey[k];
				mfey[k] = robot::max(testy);
			}
		}
	}
	if (eth >= 0.785 / 2)
		mfeth[4] = 1, mfeth[0] = 0, mfeth[1] = 0, mfeth[2] = 0, mfeth[3] = 0;
	else if (eth <= -0.785 / 2)
		mfeth[0] = 1, mfeth[1] = 0, mfeth[2] = 0, mfeth[3] = 0, mfeth[4] = 0;
	else {
		for (k = 0; k <= 4; k++) {
			if (eth < centeth[k]) {
				testth = 1 - (centeth[k] - eth) / lefteth[k];
				mfeth[k] = robot::max(testth);
			}
			else {
				testth = 1 - (eth - centeth[k]) / righteth[k];
				mfeth[k] = robot::max(testth);
			}
		}
	}

	double numv = 0.0;
	double denv = 0.0;
	double numw = 0.0;
	double denw = 0.0;

	/* for loop fills premise matrix using min operator */
	for (k = 0; k <= 4; k++) {
		for (j = 0; j <= 4; j++) {
			for (int m = 0; m <= 4; m++) {
				premv[k][j][m] = mfex[k] * mfey[j] * mfeth[m];
				numv = numv + tablev[k][j][m] * premv[k][j][m];
				denv = denv + premv[k][j][m];

				premw[k][j][m] = mfex[k] * mfey[j] * mfeth[m];
				numw = numw + tablew[k][j][m] * premw[k][j][m];
				denw = denw + premw[k][j][m];

				//std::cout << "mfex[" <<k<<"]"<<mfex[k]<< " mfey["<<j<<"]"<<mfey[j]<<std::endl;
				//std::cout<< "tablev["<<k<<"]"<<"["<<j<<"]"<<tablev[k][j]<<"tablew["<<k<<"]"<<"["<<j<<"]"<<tablew[k][j]<<std::endl;
			}
		}
	}

	s.v = numv / denv;    /* output v of Fuzzy Controller */
	s.w = numw / denw;    /* output w of Fuzzy Controller */

		if (s.v < 0)//not allow negative velocity
		{
			s.v = 0;
			s.w = 0;
		}
	std::cout << "ex: " << ex << " ey: " << ey<<endl; 
	std::cout << "xr: " << x << " yr: " << y << "thr: " << thr[i + 1] << std::endl;
	std::cout << "velocity: " << s.v << " omega: " << s.w << std::endl << std::endl;
    std::cout<<"cov_x:\t"<<a.covx<<"\tcov_y:\t"<<a.covy<<"\tcov_th:\t"<<a.covth<<"\n";
    nh.getParam("filename",filename);
	string ref_file=filename+"ref.txt";
	std::ofstream reffile(ref_file.c_str(), std::ios::binary | std::ios::app);//reference
	reffile  << x << "\t" << y << "\t" << thr[i + 1] << std::endl;
	/*std::ofstream testfile("vw.txt", std::ios::binary | std::ios::app);
	testfile << "numv: " << numv << " denv: " << denv << "numw: " << numw << "denw: " << denw << "ex: " << ex << " ey: " << ey << std::endl;*/
	return s;

}

void robot::obstacle_avoid()
{
	ros::Rate ob(1);//1Hz=1
    for(int i=0;i<512;i++)
    {
        if(isnan(laser[i]))
            continue;
        else if(isinf(laser[i]))
            continue;
        else if(laser[i]<0)
            continue;
        else if(laser[i]<0.5)
        {
            std::cout << " Robot stop !! sonar dist: " << sonardist << std::endl;
            double v = 0;
            double w = 0;
            geometry_msgs::Twist move_val;
            move_val.linear.x = v;
            move_val.angular.z = w;
            cmd_vel_pub.publish(move_val);
            ros::spinOnce();
            ob.sleep();
        }
    }
//	while ()//(sonarx[2] <= 0.5 || sonarx[3] <= 0.7 || sonarx[4] <= 0.7 || sonarx[5] <= 0.5) && ros::ok()
//	{

//		std::cout << " Robot stop !! sonar dist: " << sonardist << std::endl;
//		double v = 0;
//		double w = 0;
//		geometry_msgs::Twist move_val;
//		move_val.linear.x = v;
//		move_val.angular.z = w;
//		cmd_vel_pub.publish(move_val);
//		ros::spinOnce();
//		ob.sleep();
//	}
}
void robot::stop()//stop never awake
{

	double v = 0;
	double w = 0;
	std::cout << "Robot stop !!" << std::endl;
	geometry_msgs::Twist move_val;
	move_val.linear.x = v;
	move_val.angular.z = w;
	cmd_vel_pub.publish(move_val);
	ros::spinOnce();


}
void robot::pause()//pause and awake when sonar is closing to user
{
	int sonarcount = 0;
	while (ros::ok())//waiting for user
	{
		ros::Rate r(1);


		if (SW==1)
		{
			sonarcount++;
		}

		if (sonarcount == 2)
		{
			sonarcount = 0;
			break;
		}
		std::cout << " Robot waiting for user's command !! SW: " << SW << std::endl;
		SW = 0;
		int v = 0;
		int w = 0;
		geometry_msgs::Twist move_val;
		move_val.linear.x = v;
		move_val.angular.z = w;
		cmd_vel_pub.publish(move_val);
		ros::spinOnce();
		r.sleep();
	}
}
void robot::turn_left_45(int UTT, double k)
{
	ros::Rate r(CLOCK_SPEED);
	int i = 0;
	std::cout << "  Turning 45 degree ... " << std::endl;
	while (ros::ok() && i < UTT)
	{
		i++;
		double v = 0.0;
		double w = PI/4*CLOCK_SPEED*k / UTT;//k=1.05 for nissan
		std::cout << "  U-Turning ... " << std::endl;
		geometry_msgs::Twist move_val;
		move_val.linear.x = v;
		move_val.angular.z = w;
		cmd_vel_pub.publish(move_val);
		ros::spinOnce();
		r.sleep();
	}

}

void robot::turn_any_angle(double angle ,int UTT, double k)
{
	ros::Rate r(1);
	int i = 0;
	std::cout << "  Turning " <<angle<<" degree ... " << std::endl;
	while (ros::ok() && i < UTT)
	{
		i++;
		double v = 0.0;
		double w = angle *PI/180 * CLOCK_SPEED*k / UTT;//k=1.05 for nissan
		std::cout << "  Turning ... " << std::endl;
		geometry_msgs::Twist move_val;
		move_val.linear.x = v;
		move_val.angular.z = w;
		cmd_vel_pub.publish(move_val);
		ros::spinOnce();
		r.sleep();
	}
	robot::stop();

}

void robot::U_turn(int UTT, double k)
{
	ros::Rate r(CLOCK_SPEED);
	int i = 0;
	std::cout << "  U-Turning ... " << std::endl;
	while (ros::ok() && i < UTT)
	{
		i++;
		double v = 0.0;
		double w = PI*CLOCK_SPEED*k / UTT;//k=1.05 for nissan
		std::cout << "  U-Turning ... " << std::endl;
		geometry_msgs::Twist move_val;
		move_val.linear.x = v;
		move_val.angular.z = w;
		cmd_vel_pub.publish(move_val);
		ros::spinOnce();
		r.sleep();
	}
	//robot::pause_time(0.5);
}
void robot::pause_time(double t)
{
	ros::Rate r(1);
	int i = 0;
	std::cout << "  Stop for  " << t << " seconds ... " << std::endl;
	while (ros::ok() && i < t*4)
	{
		i++;
		double v = 0.0;
		double w = 0.0;
		geometry_msgs::Twist move_val;
		move_val.linear.x = v;
		move_val.angular.z = w;
		cmd_vel_pub.publish(move_val);
		ros::spinOnce();
		r.sleep();
	}
}
void robot::angle_simplify(int i)
{
	theta_track = (thr[i + 1] - thr[i]);
	if (theta_track > 3.14)//if theta track >180 then -360
	{
		theta_track = theta_track - 3.14 * 2;
	}
	if (theta_track < -3.14)//if theta track <-180 then +360
	{
		theta_track = theta_track + 3.14 * 2;
	}
	if (theta_track > 0.1)
	{
		robot::turn_left(i);
	}
	if (theta_track < -0.1)
	{
		robot::turn_right(i);

	}
}


void robot::turn_left(int i)
{
	ros::Rate r(CLOCK_SPEED);
	double thcount = 0.0;
	//std::cout << "delta thr: " << thr[i + 1] - thr[i] << std::endl;
	int j = 0;
	int turning_time=2;
	nh.getParam("turning_time",turning_time);
	while (ros::ok() && j < turning_time)
	{
		j++;
		double v = 0.0;
        double w= 0;
        if(theta_track<2&&theta_track>-2)
            w = theta_track*CLOCK_SPEED*0.9 / turning_time;//k=1.05 for nissan
        else
            w = theta_track*CLOCK_SPEED*1.05 / turning_time;//k=1.05 for nissan
        std::cout << "  Turning left... " << std::endl;
		geometry_msgs::Twist move_val;
		move_val.linear.x = v;
		move_val.angular.z = w;
		cmd_vel_pub.publish(move_val);
		ros::spinOnce();
		r.sleep();
	}
	//while (theta_track>0.1 && ros::ok())
	//{
	//	
	//	thcount = thcount + 0.1;
	//	double v = 0;
	//	double w = 0.1;


	//	if (thcount >theta_track*CLOCK_SPEED)
	//	{
	//		thcount = 0;
	//		break;
	//	}

	//	geometry_msgs::Twist move_val;
	//	move_val.linear.x = v;
	//	move_val.angular.z = w;
	//	cmd_vel_pub.publish(move_val);
	//	std::cout << "theta_track: " << theta_track / PIM << " w: " << w << " thcount" << thcount << std::endl;
	//	ros::spinOnce();
	//	r.sleep();

	//}

	
}
void robot::turn_right(int i)
{
	ros::Rate r(CLOCK_SPEED);
	double thcount = 0.0;
	//std::cout << "delta thr: " << thr[i + 1] - thr[i] << std::endl;
	//if ((thr[i + 1] - thr[i])<-0.1 && ros::ok())
	//{
	//	//std::cout << "ffffkkkkk thr: " << std::endl;
	//}
	int j = 0;
	int turning_time=2;
	nh.getParam("turning_time",turning_time);
	
	while (ros::ok() && j < turning_time)
	{
		j++;
		double v = 0.0;
        double w= 0;
        if(theta_track<2&&theta_track>-2)
            w = theta_track*CLOCK_SPEED*0.9 / turning_time;//k=1.05 for nissan
        else
            w = theta_track*CLOCK_SPEED*1.05 / turning_time;//k=1.05 for nissan
		std::cout << "  Turning right ... " << std::endl;
		geometry_msgs::Twist move_val;
		move_val.linear.x = v;
		move_val.angular.z = w;
		cmd_vel_pub.publish(move_val);
		ros::spinOnce();
		r.sleep();
	}
	//while (theta_track<-0.1 && ros::ok())
	//{

	//	//std::cout << "ffffkkkkksss thr: " << std::endl;

	//	thcount = thcount - 0.1;
	//	double v = 0;
	//	double w = -0.1;


	//	if (thcount <theta_track*CLOCK_SPEED)
	//	{	
	//		thcount = 0;
	//		break;
	//	}

	//	geometry_msgs::Twist move_val;
	//	move_val.linear.x = v;
	//	move_val.angular.z = w;
	//	cmd_vel_pub.publish(move_val);
	//	std::cout << "theta_track: " << theta_track / PIM << " w: " << w << " thcount" << thcount << std::endl;
	//	ros::spinOnce();
	//	r.sleep();
	//}
}
void robot::along_rightwall(double x_target,double y_target,double dist, double v, double w, double theta)
{
	int time = 0,i=0;
	double old_laser718 = laser718;
    robot::fusion(5,1,"");
	while (ros::ok() && abs(laser718 - old_laser718) < dist)//&&laser718>0.
	{
		//++;
		//robot::fusion(i,1);
	//	robot::obstacle_avoid();
		robot::send_position(a.x,a.y,"moving","moving");
		std::cout<<"Diff718: "<<(old_laser718-laser718)<<std::endl;
		old_laser718 = laser718;
		ros::Rate r(3);
		double v1 = v;
		double w1 = w;
		//int thetacount = 0;
		ros::spinOnce();
		cout << "angle:" << a.th * 180 / PI << endl;
	
		if (a.th * 180 / PI > 0.95 && laser718 < 0.58)
		{
			w1 = w1 - 0.05;
			cout << "angle:" << a.th * 180 / PI << "turn right!" << endl;
		}
		if (laser718 < 0.48)
		{
			w1 = w1 - 0.1;
			cout << "turn right!" << endl;
		}
		if (a.th * 180 / PI<-0.95 && laser718>0.8)
		{
			w1 = w1 + 0.05;
			cout << "angle:" << a.th * 180 / PI << "turn left!" << endl;
		}
		if (laser718>1)
		{
			w1 = w1 + 0.1;
			cout << "turn left!" << endl;
		}
		//std::cout << "sonar 7 = " << sonar7 << std::endl;

		//if (sonar7 < range1)//too left. range is negative. for example range =-2.4
		//{
		//	v1 = v;
		//	w1 = w - 0.05;//turn right
		//	thetacount++;
		//	if (thetacount > 5 && sonar0 - sonar7 > totalwidth)
		//	{
		//		w1 = w1 + 0.15;
		//		thetacount = 3;
		//		std::cout << "Reversed!!!" << std::endl;
		//	}
		//	std::cout << "too left , turn right..." << std::endl;
		//}
		//if (sonar7 > range2)//too right. range is negative. for example range =-2.4
		//{
		//	v1 = v;
		//	w1 = w + 0.05;//turn left
		//	thetacount--;
		//	if (thetacount < -5 && sonar0 - sonar7 > totalwidth)
		//	{
		//		w1 = w1 - 0.15;
		//		thetacount = -3;
		//		std::cout << "Reversed!!!" << std::endl;
		//	}
		//	std::cout << "too right , turn left..." << std::endl;
		//}

		geometry_msgs::Twist move_val;
		move_val.linear.x = v1;
		move_val.angular.z = w1;
		cmd_vel_pub.publish(move_val);
		r.sleep();
	}

}
void robot::along_rightwall2(double x_target, double y_target, double dist, double v, double w, double theta)
{
	int time = 0,i=0;
	double old_laser2 = laser2;
    robot::fusion(5,1,"");
	while (ros::ok() && abs(laser2 - old_laser2) < dist)//&&laser2>0.2
	{
	//	robot::obstacle_avoid();
		//i++;
		//robot::fusion(i,1);
		std::cout<<"Diff2: "<<(old_laser2-laser2)<<std::endl;
		robot::send_position(a.x,a.y,"moving","moving");
		old_laser2 = laser2;
		ros::Rate r(3);
		double v1 = v;
		double w1 = w;
		//int thetacount = 0;
		ros::spinOnce();
		cout << "angle:" << a.th * 180 / PI << endl;

		if (laser2 < 0.78)
		{
			w1 = w1 + 0.05;
			cout << "angle:" << a.th * 180 / PI << "turn left!" << endl;
		}
		if (laser2 < 0.58)
		{
			w1 = w1 + 0.1;
			cout << "turn left!" << endl;
		}
		if (laser2>0.8)
		{
			w1 = w1 - 0.05;
			cout << "angle:" << a.th * 180 / PI << "turn right!" << endl;
		}
		if (laser2>1)
		{
			w1 = w1 - 0.1;
			cout << "turn right!" << endl;
		}
		
		geometry_msgs::Twist move_val;
		move_val.linear.x = v1;
		move_val.angular.z = w1;
		cmd_vel_pub.publish(move_val);
		r.sleep();
	}

}
void robot::fusion(int i ,int amcl_modify,string path_name_x)// fuse amcl and odometry with weight.Fix the error every 10 epoch
{
	double w1 = 1;
	double w2 = 0;
	//double xot = 0;
	//double yot = 0;
	//double tho_t = 0;
	//double xat = 0;
	//double yat = 0;
	//double tha_t = 0;
	//double xm = 0;
	//double ym = 0;
	//double thm = 0;

	ros::spinOnce();

    double covx_limit=0.1;
    double covy_limit=0.1;
    double covth_limit=0.1;
    nh.getParam("covx_limit",covx_limit);
    nh.getParam("covy_limit",covy_limit);
    nh.getParam("covth_limit",covth_limit);



	double dx = o.x - xot; // x odom - x odom old
	double dy = o.y - yot;// y odom - y odom old
	double dth = o.th - tho_t;// th odom - th odom old


	double eex = a.x - xm; // amcl x -odom x
	double eey = a.y - ym;// amcl y -odom y
    double eeth = a.th - thm;//// amcl th -odom th

    double range =0.2;
//    if(i==3)
//    {
//        nh.setParam("modify_range",0.2);
//    }
    nh.getParam("modify_range",range);

    if (a.covx>(covx_limit-0.05)||a.covy>(covy_limit-0.05))//bounded the correction of AMCL (0.05m)//|| abs(eeth) > 0.053
    {
        if (eex > range)
        {
            eex = range;
            //std::cout << "ex overshoot!!" << std::endl;
        }
        if (eex < -range)
        {
            eex = -range;
            //std::cout << "ex undershoot!!" << std::endl;
        }
        if (eey > range)
        {
            eey = range;
            //std::cout << "ey overshoot!!" << std::endl;
        }
        if (eey < -range)
        {
            eey = -range;
            //std::cout << "ey undershoot!!" << std::endl;
        }

    }
//    else
//    {
//        if (eeth > 0.053)
//            eeth = 0.053;
//        if (eeth < -0.053)
//            eeth = -0.053;
//    }
//    if(abs(dx)>1 ||abs(dy)>1)
//    {
//        dx=0;
//        dy=0;
//        dth=0;
//    }




    int odom_belief_points=5;
    int odom_reset_points=30;
    nh.getParam("odom_belief_points",odom_belief_points);
    nh.getParam("odom_reset_points",odom_reset_points);
//    std::cout<<"odom reset?? "<<i%odom_reset_points<<std::endl;
    std::cout<<"Ban range ?? "<<robot::Banrange(path_name_x)<<std::endl;
    if(i==0)
    {
        xm=o.x;
        ym=o.y;
        thm=o.th;
    }

    if (!odom_ban && i>5)// || (robot::Banrange(path_n ame_x))
    {
        xm = xm + pow((pow(dx,2)+pow(dy,2)),0.5)*cos(thm+dth);// r*cos(theta)
        ym = ym + pow((pow(dx,2)+pow(dy,2)),0.5)*sin(thm+dth);// r*sin(theta)
        thm = thm + dth;// theta stay still
    }
    else if(i>0&&i<=5)
    {
        xm=xm+dx;
        ym=ym+dy;
        thm=thm+dth;
    }
    odom_ban=false;
	if (amcl_modify == 1) 
	{

		int epoch=1;
		nh.getParam("amcl_modify_epoch",epoch);
	

		// if (i % epoch == 0 && (xat != a.x || yat != a.y || tha_t != a.th))// if amcl didn't update  , don't update
        if (i % epoch == 0)// if amcl didn't update  , don't update
		{
            if (a.covx<covx_limit&&a.covy<covy_limit&&a.covth<covth_limit) //don't use amcl at first i%odom_reset_points>odom_belief_points&&(robot::Banrange(path_name_x)&&(!robot::Banrange(path_name_x))
            {
                  xm = xm + eex;
                  ym = ym + eey;
                  thm = thm +eeth;
//                thm = thm + eeth;

                std::cout << "===== amcl  modifying ===== " << std::endl << std::endl;
				std::cout << "\tex:\t" << eex << "\tey:\t" << eey << std::endl << std::endl;//"\teth:\t" << eeth 
				std::cout << "===== amcl  modifying ===== " << std::endl << std::endl;
                nh.getParam("filename",filename);
				string errf_file = filename+"odom_new.txt" ;
				std::ofstream errfixfile(errf_file.c_str(), std::ios::binary | std::ios::app);
				//errfixfile  << a.x << "\t" << a.y << "\t" << a.th << std::endl;
				errfixfile << xm << "\t" << ym << "\t" << thm << std::endl;
			
			}
		}

	}


	x_current = w1*xm+ w2*o.x; //w1=1 w2=0 initial
	y_current = w1*ym + w2*o.y;//w1=1 w2=0 initial
	theta_current = w1*thm + w2*o.th;//w1=1 w2=0 initial
									  
	xot = o.x;//old x
	yot = o.y;//old y
	tho_t = o.th;//old theta

	xat = a.x;//old amcl x
	yat = a.y;//old amcl y
	tha_t = a.th;//old amcl theta

	/*std::cout << "odom weighting: " << w1 << " amcl weighting: " << w2 << std::endl;*/
	robot::send_position(x_current, y_current, "moving", "moving");
	std::cout << "Odometry Modified: " << "x: " << x_current << " y: " << y_current << " th: " << theta_current << std::endl;
	std::cout << "Odometry      : " << "x: " << o.x << " y: " << o.y << " th: " << o.th << std::endl;
	std::cout << "AMCL          : " << "x: " << a.x << " y: " << a.y << " th: " << a.th << std::endl;

    nh.getParam("filename",filename);
	string fusion = filename+"fusion.txt" ;
	string error = filename+"error.txt" ;
	string odom_file =  filename+"odom.txt" ;
	string amclfile = filename+"amcl.txt" ;
	std::ofstream errfile(error.c_str(), std::ios::binary | std::ios::app);
	errfile  << eex << "\t" << eey << std::endl; //"\t" << eeth << std::endl;
	std::ofstream fusfile(fusion.c_str(), std::ios::binary | std::ios::app);
	fusfile  << x_current<< "\t"<< y_current<< "\t" << theta_current<< std::endl;
	std::ofstream odomfile(odom_file.c_str(), std::ios::binary | std::ios::app);
	odomfile  << o.x << "\t" << o.y<< "\t" << o.th << std::endl;
	std::ofstream amcl_file(amclfile.c_str(), std::ios::binary | std::ios::app);
	amcl_file  << a.x << "\t" << a.y << "\t" << a.th << std::endl;

//    robot::OdomPathpub(x_current,y_current,theta_current);
}
void robot::reset_fusion(std::string reset_mode)
{
    if(reset_mode=="zero")
    {
        xot=0;
        yot=0;
        tho_t=0;
        xm=0;
        ym=0;
        thm=0;
        x_current=0;
        y_current=0;
        theta_current=0;
    }
    else if(reset_mode=="amcl")
    {
        printf("reset fusion as amcl mode\n");
        xot=a.x;
        yot=a.y;
        tho_t=a.th;
        xm=a.x;
        ym=a.y;
        thm=a.th;
        x_current=a.x;
        y_current=a.y;
        theta_current=a.th;
    }
    else if(reset_mode=="odom")
    {
        printf("reset fusion as odom mode\n");
        xot=o.x;
        yot=o.y;
        tho_t=o.th;
        xm=o.x;
        ym=o.y;
        thm=o.th;
        x_current=o.x;
        y_current=o.y;
        theta_current=o.th;
    }
}

void robot::reset_position()
{
    ros::Rate r(1);
    std_msgs::Float32MultiArray odom;
    odom.data.push_back(a.x);
    odom.data.push_back(a.y);
    odom.data.push_back(a.th);
    odom_set.publish(odom);//publish pose to odometry
    ros::spinOnce();
    r.sleep();
    ros::spinOnce();
    robot::Posepub(o.x,o.y,0,o.ox,o.oy,o.oz,o.ow,o.th,0);
    odom_ban=true;

}
void robot::make_a_move(double v, double w, double t)
{
	int i = 0;

	//std::cout << "  Move as  " << v << " velocity and " << w << " angular velocity for " << t << " seconds ... " << std::endl;
	while (ros::ok() && i < t*2)
	{	
		std::cout << "  Move as  " << v << " velocity and " << w << " angular velocity for " << t << " seconds ... " << std::endl;
		i++;
		ros::Rate r(2);
		double v1 = v;
		double w1 = w;
		geometry_msgs::Twist move_val;
		move_val.linear.x = v1;
		move_val.angular.z = w1;
		cmd_vel_pub.publish(move_val);
		ros::spinOnce();
		r.sleep();
	}
}

void robot::change_map(int map)
{
		std_msgs::Int32 map_id;
		map_id.data=map;
		std::cout << "change to map "<<map<<std::endl;
		map_pub.publish(map_id);
		ros::spinOnce();
	
}
void robot::modify_elevator_y()
{
	float dist1=0.9;//min
	float dist2=1.05;//max
	float errd=0.0;
	ros::Rate r(1);
	
		cout<<"laser2:"<<laser2<<endl;
		if(laser2<dist1)//too right
		{
			errd=dist1-laser2+0.07;
			cout<<"turn left and move!"<<endl;
			robot::turn_any_angle(90,1,3);
			cout<<"laser2-1:"<<laser2<<endl;
			robot::make_a_move(errd,0,1);
			cout<<"laser2-2:"<<laser2<<endl;
			robot::turn_any_angle(-90,1,3);
			cout<<"laser2-3:"<<laser2<<endl;
			robot::stop();
		}
		else if(laser2>dist2)
		{
			errd=laser2-dist2+0.07;
			cout<<"turn right and move!"<<endl;
			robot::turn_any_angle(-90,1,3);
			robot::make_a_move(errd,0,1);
			robot::turn_any_angle(90,1,3);
			robot::stop();
		}
		else
		{
			robot::stop();
			cout<<"exit y-axis modify mode"<<endl;
			//break;
		}
		// ros::spinOnce();
		// r.sleep();
	
}
void robot::modify_elevator_x()
{
	float dist1=0.5;//min
	float dist2=0.6;//max
	while(ros::ok())
	{
		ros::Rate r(2);
		cout<<"laser360:"<<laser360<<endl;

		if(laser360<dist1)//too right
		{
			cout<<"move back!"<<endl;
			robot::make_a_move(-0.05,0,1);
			robot::stop();
		}
		else if(laser360>dist2)
		{
			cout<<"move forward!"<<endl;
			robot::make_a_move(0.05,0,1);
			robot::stop();
		}
		else
		{
			cout<<"exit x-axis modify mode"<<endl;
			robot::stop();
			break;
		}
		ros::spinOnce();
		r.sleep();
	}
}

void robot::align_elevator()
{
	int i=0;
	double w=0;
	float error=0;
	ros::Rate r(1);
	while(ros::ok())
	{
		error=laser340-laser380;
		cout<<"error="<<error<<endl;
		if (error<-0.005)
		{
			w=-0.04;//turn right
			cout<<"turn right..."<<endl;
		}
		else if(error >0.005)
		{	
			w=0.04;//turn left
			cout<<"turn left..."<<endl;
		}
		else
		{
			w=0;
			i++;
		}
		if (i>=3)
		{
			cout<<"exit align mode!"<<endl;
			break;
		}
		geometry_msgs::Twist move_val;
		move_val.linear.x = 0;
		move_val.angular.z = w;
		cmd_vel_pub.publish(move_val);
		ros::spinOnce();
		r.sleep();
	}
}

void robot::pathplanning(vector<double> len, vector<double> ang, std::string pathname ,double k,double pose_x,double pose_y,double pose_th)
{
    double initial_position[2]={pose_x,pose_y};
    double kx=0,ky=0;//length between points
    double nn1;//point amount
    vector<double> xr1;
    vector<double> yr1;
    for(int j=0;j<len.size();j++)
    {
        kx = k*cos((ang[j]+pose_th)*3.14/180);//
        ky = k*sin((ang[j]+pose_th)*3.14/180);
        nn1 = round(len[j]*(1/k));
        for(int i=0;i<nn1;i++)
        {
            xr1.push_back(initial_position[0]+i*kx);
            yr1.push_back(initial_position[1]+i*ky);
        }
        initial_position[0]+=(kx*nn1);
        initial_position[1]+=(ky*nn1);
    }
    xr1.push_back(initial_position[0]);
    yr1.push_back(initial_position[1]);
    XmlRpc::XmlRpcValue Xml_xr,Xml_yr;
    Xml_xr.setSize(xr1.size());
    Xml_yr.setSize(yr1.size());
    for(int i=0;i<xr1.size();i++)
    {
        Xml_xr[i]=xr1[i];
        Xml_yr[i]=yr1[i];
    }
    std::string n_xr="xr_"+pathname;
    std::string n_yr="yr_"+pathname;
    nh.setParam(n_xr,Xml_xr);
    nh.setParam(n_yr,Xml_yr);

    std::cout<<"complete path planning\n";

}
void robot::Custom_path(string path_name_x ,string path_name_y,int amcl_modify)
{

    vector<double> xr1;
    vector<double> yr1;

    int odom_reset_points=100;
    if( !nh.getParam(path_name_x, xr1) )
        ROS_ERROR("Failed to get xr parameter from server.");
    else
        std::cout<<"suceess reading xr1... \n";
    if( !nh.getParam(path_name_y, yr1) )
        ROS_ERROR("Failed to get xr parameter from server.");
    else
        std::cout<<"suceess reading yr1... \n";
    nh.getParam("odom_reset_points",odom_reset_points);

    int i = 0;
    int xr_size = xr1.size();

    robot::Pathpub(xr1,yr1);
    std::cout<<"Publish 4F path to rviz.. \n";

    nh.getParam("moving_rate",RRR);//double RRR = 2.5;// MOVING RATE
    ros::Rate r(RRR);
    while (ros::ok() && i < xr_size - 1)
    {

        s = robot::kinematic(i, xr1[i], yr1[i], xr1[i + 1], yr1[i + 1],amcl_modify,path_name_x);
        if (! path_follow)
        {
            if((i%odom_reset_points==0)&&(i!=0)&&amcl_modify&&(!robot::Banrange(path_name_x)))//i%odom_reset_points==0
                robot::reset_position();
            robot::angle_simplify(i);
            i++;
        }
        geometry_msgs::Twist move_val;
        move_val.linear.x = s.v;
        move_val.angular.z = s.w;
        cmd_vel_pub.publish(move_val);
        ros::spinOnce();
        send_robotpose(x_current,y_current);
        r.sleep();
    }

}


void robot::Custom_linear_path(string path_name_x ,string path_name_y,int amcl_modify)
{

    vector<double> xr1;
    vector<double> yr1;

    int odom_reset_points=100;
    if( !nh.getParam(path_name_x, xr1) )
        ROS_ERROR("Failed to get xr parameter from server.");
    else
        std::cout<<"suceess reading xr1... \n";
    if( !nh.getParam(path_name_y, yr1) )
        ROS_ERROR("Failed to get xr parameter from server.");
    else
        std::cout<<"suceess reading yr1... \n";
    nh.getParam("odom_reset_points",odom_reset_points);

    int i = 0;
    int xr_size = xr1.size();
//    robot::Pathpub(xr1,yr1);
//    std::cout<<"Publish 4F path to rviz.. \n";

    nh.getParam("moving_rate",RRR);//double RRR = 2.5;// MOVING RATE
    ros::Rate r(RRR);
    while (ros::ok() && i < xr_size - 1)
    {

        s = robot::kinematic(i, xr1[i], yr1[i], xr1[i + 1], yr1[i + 1],amcl_modify,path_name_x);
        if (! path_follow)
        {
            if( (i%odom_reset_points==0 )&&(i!=0)&&amcl_modify&&(!robot::Banrange(path_name_x)))
                robot::reset_position();
            robot::angle_simplify(i);
            i++;
        }
        geometry_msgs::Twist move_val;
        move_val.linear.x = s.v;
        move_val.angular.z = 0;
        cmd_vel_pub.publish(move_val);
        ros::spinOnce();
        r.sleep();
    }

}

void robot::Astar_path()//just for test
{
	//==========path================= 

	//double xr[] = { 0.0,0.1000,0.2000,0.3000,0.4000,0.5000,0.6000,0.7000,0.8000,0.9000,1.0000,1.1000,1.2000,1.3000,1.4000,1.5000,1.6000,1.7000,1.8000,1.9000,2.0000,2.1000,2.2000,2.3000,2.4000,2.5000,2.6000,2.7000,2.8000,2.9000,3.0000 };
	//double yr[] = { 0.0,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000 };
	//double yr[] = { 0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,-0.1000,-0.2000,-0.3000,-0.4000,-0.5000,-0.6000,-0.7000,-0.8000,-0.9000,-1.0000,-1.1000,-1.2000,-1.3000,-1.4000,-1.5000,-1.6000,-1.7000,-1.8000,-1.9000,-2.0000,-2.1000,-2.2000,-2.3000,-2.4000,-2.5000,-2.6000,-2.7000,-2.8000,-2.9000,-3.0000,-3.1000,-3.2000,-3.3000,-3.4000,-3.5000,-3.6000,-3.7000,-3.8000,-3.9000,-4.0000,-4.1000,-4.2000,-4.3000,-4.4000,-4.5000,-4.6000,-4.7000,-4.8000,-4.9000,-5.0000,-5.1000,-5.2000,-5.3000,-5.4000,-5.5000,-5.6000,-5.7000,-5.8000,-5.9000,-6.0000,-6.1000,-6.2000,-6.3000,-6.4000,-6.5000,-6.6000,-6.7000,-6.8000,-6.9000,-7.0000,-7.1000,-7.2000,-7.3000,-7.4000,-7.5000,-7.6000,-7.7000,-7.8000,-7.9000,-8.0000,-8.1000,-8.2000,-8.3000,-8.4000,-8.5000,-8.6000,-8.7000,-8.8000,-8.9000,-9.0000,-9.1000,-9.2000,-9.3000,-9.4000,-9.5000,-9.6000,-9.7000,-9.8000,-9.9000,-10.0000,-10.1000,-10.2000,-10.3000,-10.4000,-10.5000,-10.6000,-10.7000,-10.8000,-10.9000,-11.0000,-11.1000,-11.2000,-11.3000,-11.4000,-11.5000,-11.6000,-11.7000,-11.8000,-11.9000,-12.0000,-12.1000,-12.2000,-12.3000,-12.4000,-12.5000,-12.6000,-12.7000,-12.8000,-12.9000,-13.0000,-13.0707,-13.1414,-13.2121,-13.2828,-13.3536,-13.4243,-13.4950,-13.5657,-13.6364,-13.7071,-13.7778,-13.8485,-13.9192,-13.9899,-14.0607,-14.1314,-14.2021,-14.2728,-14.3435,-14.4142,-14.4849,-14.5556,-14.6263,-14.6971,-14.7678,-14.8385,-14.9092,-14.9799,-15.0506,-15.1213,-15.1920,-15.2627,-15.3335,-15.4042,-15.4749,-15.5456,-15.6163,-15.6870,-15.7577,-15.8284,-15.8991,-15.9698,-16.0406,-16.1113,-16.1820,-16.2527,-16.3234,-16.3941,-16.4648,-16.5355,-16.6062,-16.6770,-16.7477,-16.8184,-16.8891,-16.9598,-17.0305,-17.1012,-17.1719,-17.2426,-17.3134,-17.3841,-17.4548,-17.5255,-17.5962,-17.6669,-17.7376 };
	//double xr[] = { 0.0000,0.1000,0.2000,0.3000,0.4000,0.5000,0.6000,0.7000,0.8000,0.9000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0000,1.0707,1.1414,1.2121,1.2828,1.3536,1.4243,1.4950,1.5657,1.6364,1.7071 };
	//double yr[] = { 0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,-0.1000,-0.2000,-0.3000,-0.4000,-0.5000,-0.6000,-0.7000,-0.8000,-0.9000,-1.0000,-1.0707,-1.1414,-1.2121,-1.2828,-1.3536,-1.4243,-1.4950,-1.5657,-1.6364,-1.7071 };
	//XX_9F_1_path.m
	//double xr[] = { 0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000 };
	//double yr[] = { 0.0000,-0.1000,-0.2000,-0.3000,-0.4000,-0.5000,-0.6000,-0.7000,-0.8000,-0.9000,-1.0000,-1.1000,-1.2000 };


	/*double xr[] = {0.0000,0.1000,0.2000,0.3000,0.4000,0.5000,0.6000,0.7000,0.8000,0.9000,1.0000,1.1000,1.2000,1.3000,1.4000,1.5000,1.6000,1.7000,1.8000,1.9000,2.0000,2.1000,2.2000,2.3000,2.4000,2.5000,2.6000,2.7000,2.8000,2.9000,3.0000,3.1000,3.2000,3.3000,3.4000,3.5000,3.6000,3.7000,3.8000,3.9000,4.0000,4.1000,4.2000,4.3000,4.4000,4.5000,4.6000,4.7000,4.8000,4.9000,5.0000,5.1000,5.2000,5.3000,5.4000,5.5000,5.6000,5.7000,5.8000,5.9000,6.0000,6.1000,6.2000,6.3000,6.4000,6.5000,6.6000,6.7000,6.8000,6.9000,7.0000,7.1000,7.2000,7.3000,7.4000,7.5000,7.6000,7.7000,7.8000,7.9000,8.0000,8.1000,8.2000,8.3000,8.4000,8.5000,8.6000,8.7000,8.8000,8.9000,9.0000,9.1000,9.2000,9.3000,9.4000,9.5000,9.6000,9.7000,9.8000,9.9000,10.0000,10.1000,10.2000,10.3000,10.4000,10.5000,10.6000,10.7000,10.8000,10.9000,11.0000,11.1000,11.2000,11.3000,11.4000,11.5000,11.6000,11.7000,11.8000,11.9000,12.0000,12.1000,12.2000,12.3000,12.4000,12.5000,12.6000,12.7000,12.8000,12.9000,13.0000,13.1000,13.2000,13.3000,13.4000,13.5000,13.6000,13.7000,13.8000,13.9000,14.0000,14.1000,14.2000,14.3000,14.4000,14.5000,14.6000,14.7000,14.8000,14.9000,15.0000,15.1000,15.2000,15.3000,15.4000,15.5000,15.6000,15.7000,15.8000,15.9000,16.0000,16.1000,16.2000,16.3000,16.4000,16.5000,16.6000,16.7000,16.8000,16.9000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0000,17.0707,17.1414,17.2121,17.2828,17.3536,17.4243,17.4950,17.5657,17.6364,17.7071,17.7778,17.8485,17.9192,17.9899,18.0607,18.1314,18.2021,18.2728,18.3435,18.4142,18.4849,18.5556,18.6263,18.6971,18.7678,18.8385,18.9092,18.9799,19.0506,19.1213,19.1920,19.2627,19.3335,19.4042,19.4749,19.5456,19.6163,19.6870,19.7577,19.8284,19.8991,19.9698,20.0406,20.1113,20.1820,20.2527};
	double yr[] = {0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,-0.1000,-0.2000,-0.3000,-0.4000,-0.5000,-0.6000,-0.7000,-0.8000,-0.9000,-1.0000,-1.1000,-1.2000,-1.3000,-1.4000,-1.5000,-1.6000,-1.7000,-1.8000,-1.9000,-2.0000,-2.1000,-2.2000,-2.3000,-2.4000,-2.5000,-2.6000,-2.7000,-2.8000,-2.9000,-3.0000,-3.1000,-3.2000,-3.3000,-3.4000,-3.5000,-3.6000,-3.7000,-3.8000,-3.9000,-4.0000,-4.1000,-4.2000,-4.3000,-4.4000,-4.5000,-4.6000,-4.7000,-4.8000,-4.9000,-5.0000,-5.1000,-5.2000,-5.3000,-5.4000,-5.5000,-5.6000,-5.7000,-5.8000,-5.9000,-6.0000,-6.1000,-6.2000,-6.3000,-6.4000,-6.5000,-6.6000,-6.7000,-6.8000,-6.9000,-7.0000,-7.1000,-7.2000,-7.3000,-7.4000,-7.5000,-7.6000,-7.7000,-7.8000,-7.9000,-8.0000,-8.1000,-8.2000,-8.3000,-8.4000,-8.5000,-8.6000,-8.7000,-8.8000,-8.9000,-9.0000,-9.1000,-9.2000,-9.3000,-9.4000,-9.5000,-9.6000,-9.7000,-9.8000,-9.9000,-10.0000,-10.1000,-10.2000,-10.3000,-10.4000,-10.5000,-10.6000,-10.7000,-10.8000,-10.9000,-11.0000,-11.1000,-11.2000,-11.3000,-11.4000,-11.5000,-11.6000,-11.7000,-11.8000,-11.9000,-12.0000,-12.1000,-12.2000,-12.3000,-12.4000,-12.5000,-12.6000,-12.7000,-12.8000,-12.9000,-13.0000,-13.0707,-13.1414,-13.2121,-13.2828,-13.3536,-13.4243,-13.4950,-13.5657,-13.6364,-13.7071,-13.7778,-13.8485,-13.9192,-13.9899,-14.0607,-14.1314,-14.2021,-14.2728,-14.3435,-14.4142,-14.4849,-14.5556,-14.6263,-14.6971,-14.7678,-14.8385,-14.9092,-14.9799,-15.0506,-15.1213,-15.1920,-15.2627,-15.3335,-15.4042,-15.4749,-15.5456,-15.6163,-15.6870,-15.7577,-15.8284,-15.8991,-15.9698,-16.0406,-16.1113,-16.1820,-16.2527};	*/
	//double xr[] = { 0.0,0.0707,0.1414,0.2121,0.2828,0.3536,0.4243,0.4950,0.5657,0.6364,0.7071,0.7778,0.8485,0.9192,0.9899,1.0607,1.1314,1.2021,1.2728,1.3435,1.4142,1.4849,1.5556,1.6263,1.6971,1.7678,1.8385,1.9092,1.9799,2.0506,2.1213,2.1920,2.2627,2.3335,2.4042,2.4749,2.5456,2.6163,2.6870,2.7577,2.8284,2.8991,2.9698,3.0406,3.1113,3.1820,3.2527,3.3234,3.3941,3.4648,3.4648,3.4648,3.4648,3.4648,3.4648,3.4648,3.4648,3.4648,3.4648,3.4648,3.4648,3.4648,3.4648,3.4648,3.4648,3.4648,3.4648,3.4648,3.4648,3.4648,3.4648,3.4648,3.4648,3.4648,3.4648,3.4648,3.4648,3.4648,3.4648,3.4648,3.4648,3.4648,3.4648 };
	//double yr[] = { 0.0,0.0707,0.1414,0.2121,0.2828,0.3536,0.4243,0.4950,0.5657,0.6364,0.7071,0.7778,0.8485,0.9192,0.9899,1.0607,1.1314,1.2021,1.2728,1.3435,1.4142,1.4849,1.5556,1.6263,1.6971,1.7678,1.8385,1.9092,1.9799,2.0506,2.1213,2.1920,2.2627,2.3335,2.4042,2.4749,2.5456,2.6163,2.6870,2.7577,2.8284,2.8991,2.9698,3.0406,3.1113,3.1820,3.2527,3.3234,3.3941,3.4648,3.5048,3.5448,3.5848,3.6248,3.6648,3.7048,3.7448,3.7848,3.8248,3.8648,3.9048,3.9448,3.9848,4.0248,4.0648,4.1048,4.1448,4.1848,4.2248,4.2648,4.3048,4.3448,4.3848,4.4248,4.4648,4.5048,4.5448,4.5848,4.6248,4.6648,4.7048,4.7448,4.7848 };
	//==========path=================
	int i = 0;


	//int xr_size = sizeof(xr) / sizeof(xr[0]);
	ros::spinOnce();

	int xr_size = path.poses.size();
	ros::Rate r(1);
	while (ros::ok())
	{
		//std::cout << "frame_id = " << path.header.frame_id << "\tstamp\t" << path.header.stamp;
		std::cout << " xrsize" << xr_size << std::endl;//

		

        s = robot::kinematic(i, xr[i], yr[i], xr[i + 1], yr[i + 1],1,"");
		robot::angle_simplify(i);

		if (i == xr_size - 1)
		{
			break;
		}
		i++;

		geometry_msgs::Twist move_val;
		move_val.linear.x = s.v;
		move_val.angular.z = s.w;
		cmd_vel_pub.publish(move_val);
		ros::spinOnce();
		r.sleep();
	}

}
void robot::Pathpub(vector <double> xr,vector <double> yr){
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    int xr_size = xr.size();
    nav_msgs::Path path;
    //nav_msgs::Path path;
    path.header.stamp=current_time;
    path.header.frame_id="map";

    ros::Rate loop_rate(10);
    int i=0;
    while (ros::ok() && i< xr_size-1)
    {
        double x = xr[i];
        double y = yr[i];
        double th = 0.0;

        if(i>=1)
            th = atan2((yr[i+1] - yr[i]), (xr[i+1] - xr[i]));
        i++;
        current_time = ros::Time::now();
        //compute odometry in a typical way given the velocities of the robot
//        double dt = (current_time - last_time).toSec();
//        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
//        double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
//        double delta_th = vth * dt;

//        x += delta_x;
//        y += delta_y;
//        th += delta_th;


        geometry_msgs::PoseStamped this_pose_stamped;
        this_pose_stamped.pose.position.x = x;
        this_pose_stamped.pose.position.y = y;

        geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(th);
        this_pose_stamped.pose.orientation.x = goal_quat.x;
        this_pose_stamped.pose.orientation.y = goal_quat.y;
        this_pose_stamped.pose.orientation.z = goal_quat.z;
        this_pose_stamped.pose.orientation.w = goal_quat.w;

        this_pose_stamped.header.stamp=current_time;
        this_pose_stamped.header.frame_id="map";
        path.poses.push_back(this_pose_stamped);




        last_time = current_time;

    }
    path_pub.publish(path);
    ros::spinOnce();               // check for incoming messages
    loop_rate.sleep();

}

void robot::OdomPathpub(double x,double y,double th){
    current_time = ros::Time::now();
//    last_time = ros::Time::now();


    nav_msgs::Path path;
    //nav_msgs::Path path;
    path.header.stamp=current_time;
    path.header.frame_id="map";

    ros::Rate loop_rate(1);

    current_time = ros::Time::now();


    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = x;
    this_pose_stamped.pose.position.y = y;

    geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(th);
    this_pose_stamped.pose.orientation.x = goal_quat.x;
    this_pose_stamped.pose.orientation.y = goal_quat.y;
    this_pose_stamped.pose.orientation.z = goal_quat.z;
    this_pose_stamped.pose.orientation.w = goal_quat.w;

    this_pose_stamped.header.stamp=current_time;
    this_pose_stamped.header.frame_id="map";
    path.poses.push_back(this_pose_stamped);




//        last_time = current_time;


    odom_path_pub.publish(path);
    ros::spinOnce();               // check for incoming messages
    loop_rate.sleep();

}
