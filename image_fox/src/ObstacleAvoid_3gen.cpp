#include "obstacle_avoid_3gen.h"

using namespace std;

void obstacle_avoid::rm_publish(int rm)
{
    std_msgs::Int32 robot_motion;
    robot_motion.data = rm;
    RM_pub.publish(robot_motion);
    //printf("Robot_motion has published ...\n");

    ros::spinOnce();
}
void obstacle_avoid::vel_controller(double r ,double Vel_c)
{
    float d1_max=0.45;
    ros::param::get(NameSpace+"/d1_max", d1_max);
    // nh.getParam("d1_max",d1_max);
    if ( r == 0 )
    {
        speed.left=Vel_c;
        speed.right=-Vel_c;
    }
    speed.left =1/r*(r+d1_max)*Vel_c;
    speed.right=1/r*(r-d1_max)*Vel_c;
    cout<<"vL="<<speed.left<<" "<<"vR="<<speed.right<<endl;
    if (speed.left>speed.right)
        cout<<"Turn right"<<endl;
    else if(speed.left<speed.right)
        cout<<"Turn left"<<endl;
    else
        cout<<"Go Straight"<<endl;
}

void obstacle_avoid::vel_publisher(twowheel speed)
{
    int M2=0,M3=0;
    int laggy=100;
    bool moving_lock=0;
    M2=(int)(speed.left *Ratio * 60 / (2 * PI*R));
    M3=(int)(speed.right *Ratio * 60 / (2 * PI*R));

    ros::param::get(NameSpace+"/laggy", laggy);
    ros::param::get(NameSpace+"/moving_lock", moving_lock);
    // nh.getParam("laggy",laggy);
    // nh.getParam("moving_lock",moving_lock);
    rs[0] = M2;//Motor2 speed
    rs[1] = M3;//Motor3 speed
    
    if(!moving_lock)
    { 
        std_msgs::Int32MultiArray robot_speed;
    
        if((abs(oldM2-M2)>laggy)&&(abs(oldM3-M3)>laggy))
        {
            robot_speed.data.clear();
            robot_speed.data.push_back(rs[0]);
            robot_speed.data.push_back(rs[1]);

            RS_pub.publish(robot_speed);
            obstacle_avoid::rm_publish(1);//set motion as forward
        }
        //printf("Robot_speed has published ...\n");
        ros::spinOnce();
    }
    oldM2=M2;//update old vel
    oldM3=M3;	

}


  void obstacle_avoid::CropBlocks()//Crop original image into 9 blocks and calc obstacle points number.
  {
    int px11=15;
    int px1=px11,py1=40;
    float t1=0.2;
    int t2_o=1500;
    int t2_c=160;
    int t2_oh=4100;
    int px=215;
    int py=50;
    int px2=px1+px, py2=py1+py;


    ros::param::get(NameSpace+"/px1", px11);
    ros::param::get(NameSpace+"/py1", py1);
    ros::param::get(NameSpace+"/px", px);
    ros::param::get(NameSpace+"/py", py);
    ros::param::get(NameSpace+"/t1", t1);
    ros::param::get(NameSpace+"/t2_c", t2_c);
    ros::param::get(NameSpace+"/t2_o", t2_o);
    ros::param::get(NameSpace+"/t2_oh", t2_oh);

    // nh.getParam("px1",px11);
    // nh.getParam("py1",py1);
    // nh.getParam("px",px);
    // nh.getParam("py",py);
    // nh.getParam("t1",t1);
    // nh.getParam("t2_c",t2_c);//center blocks' threshold
    // nh.getParam("t2_o",t2_o);//other blocks' threshold
    // nh.getParam("t2_oh",t2_oh);//other blocks' threshold
    px1=px11;
    
    for (int i=0;i<3;i++)
    {
        for (int j=0;j<3;j++) 
        {

          cv::Rect myROI(px1, py1, px, py);// Setup a rectangle to define your region of interest (x,y,width,height)
          cv::Mat croppedImage = img2(myROI);// Crop the full image to that image contained by the rectangle myROI
          cv::Scalar ss = cv::sum( croppedImage );
          double s = ss.val[0];
          //std::cout<<"There're "<<s<<" points in the blocks!"<<"\n";
          //std::cout<<"("<<i<<","<<j<<"): "<<s<<"\n";
          drive_hard=0;
	    if ((j==1) && (s>=t2_c))
            Blocks[i][j]=1;
        else if (s>=t2_o)
        {
            Blocks[i][j]=1;
	        if (s>=t2_oh)
	    	    drive_hard=1;  
	    }
        else
            Blocks[i][j]=0;
          // cv::imshow(OPENCV_WINDOW,croppedImage);
        px1=px1+px;
        }  
        py1=py1+py;
        px1=px11;
    }
  }


void obstacle_avoid::action_launcher()
{
    //Initial position (left upper point) 


    int px=215;// px= cols of a block
    int py=50;// py= rows of a block
    int m=3,n=3;//divide into 3*3 blocks
    int act=0;
    int Num[3];
    int od=0;

    ros::param::get(NameSpace+"/px", px);
    ros::param::get(NameSpace+"/py", py);
    // nh.getParam("px",px);
    // nh.getParam("py",py);
    obstacle_avoid::CropBlocks();
    
    for (int i=0;i<3;i++)
      {
        for (int j=0;j<3;j++)
        {
            std::cout<<Blocks[i][j]<<",";
        }
        Num[i] =Blocks[i][0]*4+Blocks[i][1]*2+Blocks[i][2]*1;// Giving Num[]=Grid A ,Grid B, Grid C, by using Binary coding
        cout<<"Num="<<Num[i]<<"\n";
        std::cout<<"\n";
            
      }

       
    if( ( (Num[0]==0) && (Num[1]==0) && (Num[2]==0) ) )
        //[0][0][0]
        act = 1; //Go straight
    else if ((Num[2]== 5 ||Num[2]== 7))//(Num[0]== 5 ||Num[0]== 7) || (Num[1]== 5 ||Num[1]== 7) || 
        //[1][1][1] or [1][0][1]
        act = 2; //Turn Right or Left (In situ)
    else if ((1<=Num[2]&&Num[2]<=4)||(Num[2]==6)){
        // other condition, first consider Grid C
        act = 3;// Turn left of right
        od = Num[2]; //return obstacle distribution
        cout<<"od="<<od<<"\n";
    }
    else if ((1<=Num[1]&&Num[1]<=4)||(Num[1]==6)){
        // other condition, secondly consider Grid B
        act = 1;// go straight
        od = Num[1];//return obstacle distribution
        cout<<"od="<<od<<"\n";
    }
    else if ((1<=Num[0]&&Num[0]<=4)||(Num[0]==6)){
        // other condition, last consider Grid A
        act = 1;// go straight
        od = Num[0];//return obstacle distribution
        cout<<"od="<<od<<"\n";
    }
    else
	{
	    act = 1; //Go straight
    }
    obstacle_avoid::action( act, od, px, py);
}
void obstacle_avoid::action(int act, int od ,int px,int py)
{
    int px11=15;
    int px1=px11,py1=40;
    float t1=0.2;
    int t2_o=8000;
    int t2_c=500;
    int px2=px1+px, py2=py1+py;

    ros::param::get(NameSpace+"/py1", py1);
    ros::param::get(NameSpace+"/px1", px1);
    ros::param::get(NameSpace+"/t1", t1);
    ros::param::get(NameSpace+"/t2_c", t2_c);
    ros::param::get(NameSpace+"/t2_o", t2_o);

    // nh.getParam("py1",py1);
    // nh.getParam("px1",px11);
    // nh.getParam("t1",t1);
    // nh.getParam("t2_c",t2_c);
    // nh.getParam("t2_o",t2_o);
    px1=px11;
    int center = px1+px*1.5;
    switch(od)
    {
        //calc the distance of center obstacle
 
        case 1:
            //dis=obstacle_avoid::Truthdepth(px1+2*px,py1+2*py-(act-3),px,py,center);  //[0][0][1] 223
            // dis=obstacle_avoid::Truthdepth((int)px*2.5+px1,py1+3*py-(act-3)*py,center);  //[0][0][1] 223
 	        dis=obstacle_avoid::DepthTable((act-3),4);            
            break;
        case 2:
            dis=obstacle_avoid::DepthTable((act-3),2);    
            //dis=obstacle_avoid::Truthdepth(px1+px,py1+2*py-(act-3),px,py,center);  //[0][1][0] 138
            // dis=obstacle_avoid::Truthdepth((int)px*1.5+px1,py1+3*py-(act-3)*py,center); //[0][1][0] 138
            break;
 	            
        case 3:
            //dis=obstacle_avoid::Truthdepth(px1+px,py1+2*py-(act-3),px*2,py,center);  //[0][1][1] 138
            // dis=obstacle_avoid::Truthdepth(px*2+px1, py1+3*py-(act-3)*py,center); //[0][1][1] 180
 	        dis=obstacle_avoid::DepthTable((act-3),3);            
            break;
        case 4:
            //dis=obstacle_avoid::Truthdepth(px1,py1,px,py+2*py-(act-3),center); //[1][0][0] 
	        dis=obstacle_avoid::DepthTable((act-3),0);            
            // dis=obstacle_avoid::Truthdepth((int)px/2+px1 , py1+3*py-(act-3)*py,center); //[1][0][0] 53
            break;
        case 6:
            //dis=obstacle_avoid::Truthdepth(px1,py1,2*px,py+2*py-(act-3),center); //[1][1][0]
 	        dis=obstacle_avoid::DepthTable((act-3),1);            
            // dis=obstacle_avoid::Truthdepth(px+px1, py1+3*py-(act-3)*py,center);  //[1][1][0] 95
            break;
        default:
            break;

    }
    
    switch(act)
    {
        case 1://Go straight
            {
                speed.left=Vel_c;
                speed.right=Vel_c;
                float r=radius_adjust(dis);
                cout<<"[act1] Go straight"<<"\n";
                obstacle_avoid::vel_publisher(speed);
            }
            break;
        case 2://Turn Right or Left (In situ)
            {
                float r=radius_adjust(dis);
                if(act_old == 1||r>0){ 
                    speed.left=Vel_c;
                    speed.right=-Vel_c;//turn right
                }
		else//turn left
		{
		    speed.left=-Vel_c;
		    speed.right=Vel_c;
		}
                cout<<"[act2] Turn right or left(In situ)"<<"\n";
                obstacle_avoid::vel_publisher(speed);
            }
            break;
        case 3 ://  Grid C  rotation is according to the distance to obstacle
            {
                float r=radius_adjust(dis);
                cout<<"[act3] (Grid C) "<<"\n";
                obstacle_avoid::vel_controller(r,Vel_c);
                obstacle_avoid::vel_publisher(speed);
            }
            break;
        case 4 ://  Grid B  rotation is according to the distance to obstacle
            {
                float r=radius_adjust(dis);
                cout<<"[act4] (Grid B) "<<"\n";
                obstacle_avoid::vel_controller(r,Vel_c);
                obstacle_avoid::vel_publisher(speed);
            }
            break;
        case 5 ://  Grid A  rotation is according to the distance to obstacle
            {
                float r=radius_adjust(dis);
                cout<<"[act5] (Grid A)"<<"\n";
                obstacle_avoid::vel_controller(r,Vel_c);
                obstacle_avoid::vel_publisher(speed);
            }
            break;
        default:
            break;

        act_old=act; //update old act

    }
}
float obstacle_avoid::radius_adjust(Distance dis)
{
    float r1=0.3,r2=0.5,d2_min=0.3,d2_s=0.5,d1_max=0.45;
    float r=0;
    ros::param::get(NameSpace+"/r1", r1);
    ros::param::get(NameSpace+"/r2", r2);
    ros::param::get(NameSpace+"/d2_min", d2_min);
    ros::param::get(NameSpace+"/d1_max", d1_max);
    // nh.getParam("r1",r1);
    // nh.getParam("r2",r2);
    // nh.getParam("d2_min",d2_min);
    // nh.getParam("d1_max",d1_max);
    if ((dis.d1<=0)&&(dis.d2>d2_min)&&(!drive_hard)){
        cout<<"turn righttt"<<endl;
        if ((d1_max+dis.d1)>=0.0){
            cout<<"softer!!"<<endl;
            r = r1*0.5*(dis.d1-d1_max+(pow(dis.d2,2)-pow(W/2,2))/(d1_max+dis.d1));//r<0.5*(d1-L/2+(pow(dis.d2,2)-pow(W/2,2))/(L/2+dis.d1))
        }
        else if((d1_max+dis.d1>0)&&(drive_hard)){
        //     r = r2*0.5*(dis.d1-L/2+(pow(dis.d2,2)-pow(W/2,2))/(L/2+dis.d1));//r>=0
            r = r2*0.5*(dis.d1-L/2+(pow(dis.d2,2)-pow(W/2,2))/(L/2+dis.d1));//r<0.5*(d1-L/2+(pow(dis.d2,2)-pow(W/2,2))/(L/2+dis.d1))
            cout<<"harder!!"<<endl;
        
        }
        else 
            r = 999; 
    }
    else if ((dis.d1>=0)&&(dis.d2>d2_min)&&(!drive_hard)){
        cout<<"turn lefttt"<<endl;     
        if ((d1_max-dis.d1)>=0.0){
            r = r1*-0.5*(-dis.d1-d1_max+(pow(dis.d2,2)-pow(W/2,2))/(d1_max-dis.d1));//r<0.5*(d1-L/2+(pow(dis.d2,2)-pow(W/2,2))/(L/2+dis.d1))    
             cout<<"softer!!"<<endl;
        }
        else if((d1_max-dis.d1>0)&&(drive_hard)){
           r = r2*-0.5*(-dis.d1-L/2+(pow(dis.d2,2)-pow(W/2,2))/(L/2-dis.d1));//r>=0
         cout<<"harder!!"<<endl;
         }
        else 
            r = -999;
    }
    else
        r=999;
    cout<<"d1="<<dis.d1<<"\n";
    cout<<"d2="<<dis.d2<<"\n";
    cout<<"r="<<r<<"\n";
    return r;
}


Distance obstacle_avoid::Truthdepth(int px1,int py1,int px,int py,int center) 
{
    //float h_center=no_obs_depth[Col/2+y*Col];//no obstacle center depth
    //float h_no_obs=no_obs_depth[x+y*Col];//no obstacle depth
    // float h = depth_data[x+y*Col];
    int x = px1+(int)(px/2);
    int y = py1+(int)(py/2);
    
    
    float dd = avg_depth[y];
    float m = 0.6715;
     
    cv::Rect myROI(px1, py1, px, py);// Setup a rectangle to define your region of interest (x,y,width,height)
    cv::Mat Boomat = img2(myROI);// Crop boolin mat
    cv::Mat Dmat = img3(myROI);// Crop depth
    cv::Mat Result = Dmat.mul(Boomat);// Result of multiplication 
    cv::Scalar tempR = cv::mean( Result );//calc mean 
    float h = tempR.val[0]; //mean of Result


    dis.d1 = m*dd/215*(x-center);// if obstacle left  -> negative  
    
    if( isnan(h))//process nan problem
        h=dd;
    else if(h>20)
        h=20; //process infinite problem
    else if(h<0)
        h=dd;//process negative problem
    

    float hcos = pow(pow(h,2)-pow(dis.d1,2),0.5);

    dis.d2 = -0.19+hcos*cos(theta*PI/180);//center to camera = 19cm
    

    cout<<"x: "<<x<<" y: "<<y<<"\n";
    cout<<"d1 ="<<dis.d1<<" d2 ="<<dis.d2<<endl;
    cout<<" hcos ="<<hcos<<" h ="<<h<<endl;
    return dis;
}

Distance obstacle_avoid::DepthTable(int grid,int Num)
{
   float d1_table[3][5]={ {-41.5,-22.5,0,22.5,41.5},
			              {-52.5,-29.5,0,29.5,52.5},
			              {-72,-36.5,0,36.5,72} };

   float d2_table[3]={79,111,150.5};

   dis.d1=0.01*d1_table[grid][Num];
   dis.d2=0.01*d2_table[grid];
   cout<<"d1 ="<<dis.d1<<" d2 ="<<dis.d2<<endl;
   return dis;
}

int obstacle_avoid::obstacle_detect()
{
    //Initial position (left upper point) 


    int px=215;// px= cols of a block
    int py=50;// py= rows of a block
    int m=3,n=3;//divide into 3*3 blocks
    int act=0;
    int Num[3];
    int od=0;

    ros::param::get(NameSpace+"/px", px);
    ros::param::get(NameSpace+"/py", py);
    // nh.getParam("px",px);
    // nh.getParam("py",py);
    obstacle_avoid::CropBlocks();
    
    for (int i=0;i<3;i++)
      {
        for (int j=0;j<3;j++)
        {
            std::cout<<Blocks[i][j]<<",";
        }
        Num[i] =Blocks[i][0]*4+Blocks[i][1]*2+Blocks[i][2]*1;// Giving Num[]=Grid A ,Grid B, Grid C, by using Binary coding
        cout<<"Num="<<Num[i]<<"\n";
        std::cout<<"\n";
            
      }

       
    if( ( (Num[0]==0) && (Num[1]==0) && (Num[2]==0) ) )
        //[0][0][0]
        act = 1; //Go straight
    else if ((Num[2]== 5 ||Num[2]== 7))//(Num[0]== 5 ||Num[0]== 7) || (Num[1]== 5 ||Num[1]== 7) || 
        //[1][1][1] or [1][0][1]
        act = 2; //Turn Right or Left (In situ)
    else if ((1<=Num[2]&&Num[2]<=4)||(Num[2]==6)){
        // other condition, first consider Grid C
        act = 3;// Turn left of right
        od = Num[2]; //return obstacle distribution
        cout<<"od="<<od<<"\n";
    }
    else if ((1<=Num[1]&&Num[1]<=4)||(Num[1]==6)){
        // other condition, secondly consider Grid B
        act = 1;// go straight
        od = Num[1];//return obstacle distribution
        cout<<"od="<<od<<"\n";
    }
    else if ((1<=Num[0]&&Num[0]<=4)||(Num[0]==6)){
        // other condition, last consider Grid A
        act = 1;// go straight
        od = Num[0];//return obstacle distribution
        cout<<"od="<<od<<"\n";
    }
    else
	{
	    act = 1; //Go straight
    }
    return act;
}

