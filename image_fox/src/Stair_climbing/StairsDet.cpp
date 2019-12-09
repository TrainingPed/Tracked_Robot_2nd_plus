#include "ros/ros.h"
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"
#include  <cmath>
#include  <string>
#include "StairsDet.h"
#define width 672
#define height 376
int t1=2,t2=350,t3=200,t4=200,cnt_row=200;
bool rgb_imgshow=0,depth_imgshow=0;
cv::Mat img1(col,row,CV_8UC3);//save the rgb image
cv::Mat img2(col,row,CV_8UC3);
cv::Mat grayimg;
cv::Mat dst1,dst2,dst3;
int StairDetection::BubbleSort(int array_size, int *array)
{

    int t, temp[array_size-1];
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
    // std::cout << "Sort array should be:\n";
    // for (int k = 0; k < 5; k++)
    // {
    // 	std::cout << temp[k] << ",";
    // }
    // std::cout << "\n";
    return temp[array_size / 2];
}

void StairDetection::vodom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    vox = msg->pose.pose.position.x;
    voy = msg->pose.pose.position.y;
    tf::Pose pose;
    tf::poseMsgToTF(msg->pose.pose, pose);
    tf::Matrix3x3(pose.getRotation()).getRPY(roll,pitch,yaw);
//    std::cout<<"pitch="<<pitch<<" \n";

}
void StairDetection::test()
{	
	Depth_Arr();
	int a=depthMD(width/2,height/2);
	// for(int ii=0;ii<width;ii++) 
	// {
	// 	for(int jj=1;jj<height;jj++) 
	// 	{
	// 		if((abs(depthMD(ii,jj-1)-depthMD(ii,jj))>200)&&(abs(depthMD(ii,jj-1)-depthMD(ii,jj))<350)&&(depthMD(ii,jj-1)!=0)&&(depthMD(ii,jj)!=0))
	// 		{
	// 			c=1;
	// 			ptr2[ii][jj]=c;

	// 		}
	// 	}
	// }
	ros::Rate r(2);
	while(ros::ok())
	{
		if(cv::waitKey(33) == 27)//press esc to escape
		{ 
			break;
		}
		Depth_Arr();
		a=depthMD(width/2,height/2);
		printf("data=%d\n",a);
		// cv::imshow(OPENCV_WINDOW,img1);//show result
		ros::spinOnce();
		r.sleep();
	}
/*		
	cv::Mat img(480,640,CV_16UC1); 
	cv::namedWindow("array",CV_WINDOW_AUTOSIZE);

	for(int y=0;y < 480;y++)//480 height
	{
		for(int x=0;x < 640;x++)//640 width
		{
			img.at<short int>(y,x)=depthMD(x,y)*5;
		}
	}
		
	cv::imshow("array",img);	
	cv::waitKey(10);
*/
}

void StairDetection::SD_Start()
{
    int xxx=0,yyy=0;
    ros::param::get(NameSpace+"/xxx",xxx);
    ros::param::get(NameSpace+"/yyy",yyy);
	int midxy[2];
	int i=0,j=0,x=0,y=0,mid_x=0,mid_y=0,sw,mid_x2=0,mid_y2=0;
	int i4=0,j4=0,x4=0,sw4=0,mid_x4=0,mid_y4=0;
	int i5=0,j5=0,x5=0,y5=0,sw5=0,mid_x5=0,mid_y5=0;
	int i7=0,j7=0,x7=0,y7=0,sw7=0,mid_x7=0,mid_y7=0;
	int d21_temp=0,d21_2=0,d51=0,d52=0,d71=0,d72=0;
    int width_temp,width_temp2;
	std::vector <int> d21_avg;
	d21_avg.assign(3,0);
    std_msgs::Int32 d21,cnt21,d4,d5,d7,d,md21;
    std_msgs::Int32MultiArray mid,mid21,mid5,mid7;
	ros::Rate r(2);
		

	while(ros::ok())
	{
		if(cv::waitKey(33) == 27)//press esc to escape
		{ 
			break;
        }
        mid.data.clear();
        mid21.data.clear();
        mid5.data.clear();
        mid7.data.clear();
        Depth_Arr();
        edge_find(dst2,&mid_x,&mid_y,&i,&j,&x,&sw);
        //        depth_find(22,&mid_x2,&mid_y2,&mid_x,&mid_y,&i,&j,&x,&sw);
        // std::cout<<"mid44\n";
//        depth_find_4(4,&mid_x4,&mid_y4,&i4,&j4,&x4,&sw4);

        depth_find_5(22,&mid_x5,&mid_y5,&i5,&j5,&x5,&sw5);
        depth_find_7(7,&mid_x7,&mid_y7,&i7,&j7,&x7,&sw7);
        // std::cout<<"mid74\n";
        ros::param::get(NameSpace+"/cnt_row",cnt_row);
        width_temp=width;
        for(int ii = 0 ; ii<width;ii++)
        {
            if (depthMD(ii,cnt_row)<=0)
            {
                width_temp--;
            }
            cnt21.data += depthMD(ii,cnt_row);
        }
//        width_temp2=(width-76)*10;
        ros::param::get(NameSpace+"/xxx",xxx);
        ros::param::get(NameSpace+"/yyy",yyy);
        int depth_stack[width*10];
        int n_reading=0;
        for(int j2=yyy-5;j2<yyy+5;j2++)
        {
            for(int jj=0;jj<(width);jj++)//for(int jj=38;jj<(width-76);jj++)
            {
                depth_stack[n_reading]=depthMD(jj,j2);
                n_reading++;
            }
        }
        md21.data = BubbleSort(width*10,depth_stack);
//        md21.data=depthMD(xxx,yyy);
        printf("xxx,yyy = %d,%d\n",xxx,yyy);
        printf("DesirePoint =  %d\n",md21.data);
        cnt21.data = (int)(cnt21.data/width_temp);

        if(mid_x!=0&&mid_y!=0)
        {
            d21_temp = atan2((double)(j-sw),(double)(x-i))*1000;
            printf("Horozontal Stair Slope = %f,%d \n",atan2((double)(j-sw),(double)(x-i)),d21_temp);
            d21.data=d21_temp;
//            d21_temp = depthMD(i,mid_y-20)+depthMD(i,mid_y-40)+depthMD(i,mid_y-60);
//            d21_2 = depthMD(x,mid_y-20)+depthMD(x,mid_y-40)+depthMD(x,mid_y-60);
//            printf("d21_left = %d , d21_right = %d\n\n",d21_temp,d21_2);
//            d21.data=d21_temp/3-d21_2/3;//increase sensitivity
        }
        else
            d21.data=0;

//        d21_avg.push_back(d21_temp);
//        if (d21_avg.size()>3)
//        {
//            d21_avg.erase(d21_avg.begin());
//        }
//        d21.data  = (d21_avg[0]+d21_avg[1]+d21_avg[2])/3;// average result
//         std::cout<<"d21 at ("<<i<<" , "<<mid_y-20<<") = "<<depthMD(i,mid_y-20)<<"\n";
//         std::cout<<"d21 at ("<<i<<" , "<<mid_y-40<<") = "<<depthMD(i,mid_y-40)<<"\n";
//         std::cout<<"d21 at ("<<i<<" , "<<mid_y-60<<") = "<<depthMD(i,mid_y-60)<<"\n";
//         std::cout<<"d21 at ("<<x<<" , "<<mid_y-20<<") = "<<depthMD(x,mid_y-20)<<"\n";
//         std::cout<<"d21 at ("<<x<<" , "<<mid_y-40<<") = "<<depthMD(x,mid_y-40)<<"\n";
//         std::cout<<"d21 at ("<<x<<" , "<<mid_y-60<<") = "<<depthMD(x,mid_y-60)<<"\n\n";

//        pub mode21 middle position and line points.


        if(mid_x4==0||mid_y4==0)
        {
            d4.data=0;
        }
//        else if(md21.data>300)
//            d4.data=0;
        else
        {
            d4.data=((depthMD(x4+20,j4+30)+depthMD(x4+20,j4+20)+depthMD(x4+20,j4+40))/3-(depthMD(i4-20,sw4+30)+depthMD(i4-20,sw4+20)+depthMD(i4-20,sw4+40))/3);
        }
        // mode 5~7 mid position
        mid5.data.push_back(mid_x5);
        mid5.data.push_back(mid_y5);
        mid7.data.push_back(mid_x7);
        mid7.data.push_back(mid_y7);
        d.data=depthMD(mid_x5,mid_y5+5);
        d51=(depthMD(x5+30,j5+30)+depthMD(x5+30,j5+20))/2;
        d52=(depthMD(i5-30,sw5+30)+depthMD(i5-30,sw5+20))/2;
        d71=(depthMD(x7+20,j7+15)+depthMD(x7+20,j7+20)+depthMD(x7+20,j7+10))/3;
        d72=(depthMD(i7-20,sw7+15)+depthMD(i7-20,sw7+20)+depthMD(i7-20,sw7+10))/3;

        if(mid_x5==0||mid_y5==0)
        {
            d5.data=0;
            d7.data=0;
//            ROS_INFO("Can't found center of stair ...");
        }
        else
        {
            d5.data=d51-d52;
            d7.data=d71-d72;
//            ROS_INFO("d5 & d7 publish!");
        }

        mid.data.push_back(mid_x);
        mid.data.push_back(mid_y);
        mid21.data.push_back(mid_x);
        mid21.data.push_back(mid_y);
        mid21.data.push_back(i);
        mid21.data.push_back(j);
        mid21.data.push_back(x);
        mid21.data.push_back(sw);
        //print important measurement
        ROS_INFO("21:(%d,%d),d21:%d,cnt21:%d\n",mid_x,mid_y,d21.data,cnt21.data);
//        ROS_INFO("4:(%d,%d),d4:%d",mid_x4,mid_y4,d4.data);
//        ROS_INFO("5:(%d,%d),d5:%d",mid_x5,mid_y5,d5.data);
//        ROS_INFO("7:(%d,%d),d7:%d",mid_x7,mid_y7,d7.data);
//        ROS_INFO("d:%d",d.data);
        std::cout<<"pitch="<<pitch*180/3.14<<" \n";
        std::cout<<"roll="<<roll*180/3.14<<" \n";
        std::cout<<"yaw="<<yaw*180/3.14<<" \n\n";
        //pub
        pub_21MD.publish(md21);
        pub_d21.publish(d21);
        pub_d4.publish(d4);
        pub_d.publish(d);
        pub_d5.publish(d5);
        pub_d7.publish(d7);
        pub_cnt21.publish(cnt21);
        pub_mid.publish(mid);
        pub_mid21.publish(mid21);
        pub_mid5.publish(mid5);
        pub_mid7.publish(mid7);
        cv::circle(dst3, cv::Point(mid_x,mid_y),1 ,cv::Scalar(0,0,255),  2,  8,  0);
        //        cv::circle(img1, cv::Point(mid_x,mid_y),1 ,cv::Scalar(0,0,255),  2,  8,  0);

        ros::param::get(NameSpace+"/rgb_imgshow",rgb_imgshow);

        // std::cout<<"NameSpace ="<<NameSpace<<"\n";
        // std::cout<<"depth_imgshow ="<<depth_imgshow<<"\n";

        if (rgb_imgshow)
            cv::imshow(OPENCV_WINDOW,img1);//show result
        ros::param::get(NameSpace+"/depth_imgshow",depth_imgshow);

        if (depth_imgshow)
            cv::imshow(OPENCV_WINDOW2,dst3);//show result

        ros::spinOnce();
        r.sleep();
    }
}    

void StairDetection::rgbCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
     cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
   img1 = cv_ptr->image;//temp mat
  
//    cv::cvtColor( img1, grayimg, CV_BGR2GRAY);
//    GaussianBlur(grayimg, grayimg, cv::Size(3,3), 0, 0);
//    Canny(grayimg, dst1, 50, 150);
//    cv::cvtColor(dst1,dst2, CV_GRAY2BGR);

// Houghline
//    std::vector<cv::Vec2f> lines;
//    HoughLines(dst1, lines, 1, CV_PI/180, 150);
//    for( size_t i = 0; i < lines.size(); i++ )
// 	{
// 		float rho = lines[i][0], theta = lines[i][1];
// 		cv::Point pt1, pt2;
// 		double a = cos(theta), b = sin(theta);
// 		double x0 = a*rho, y0 = b*rho;
// 		pt1.x = cvRound(x0 + 1000*(-b));
// 		pt1.y = cvRound(y0 + 1000*(a));
// 		pt2.x = cvRound(x0 - 1000*(-b));
// 		pt2.y = cvRound(y0 - 1000*(a));
// 		cv::line( dst2, pt1, pt2, cv::Scalar(0,0,255), 3, CV_AA);
// 	}

//   std::vector<cv::Vec4i> lines;
//   HoughLinesP(dst1, lines, 1, CV_PI/180, 150, 50, 10 );
//   for( size_t i = 0; i < lines.size(); i++ )
//   {
//     cv::Vec4i l = lines[i];
//     cv::line( dst2, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, CV_AA);
//   }

   
}
void StairDetection::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr2;
  try
  {
     cv_ptr2 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

	ros::param::get(NameSpace+"/t1",t1);
    ros::param::get(NameSpace+"/t2",t2);
	ros::param::get(NameSpace+"/t3",t3);
	img2 = cv_ptr2->image;//temp mat
	// cv::imshow(OPENCV_WINDOW2,img2);
	// GaussianBlur(grayimg, grayimg, cv::Size(3,3), 0, 0);
    
	
	img2.convertTo(dst1,CV_8UC1,255/t1,0);//temp?
	// cv::imshow(OPENCV_WINDOW2,dst1);
	Canny(dst1, dst2, t2, t3);
    cv::dilate(dst2,dst2,cv::Mat(),cv::Point(-1,-1),3);
    cv::erode(dst2,dst2,cv::Mat(),cv::Point(-1,-1),2);


	cv::cvtColor(dst2,dst3, CV_GRAY2BGR);

  	// for(int ii=0;ii<width;ii++) 
	// {
	// 	for(int jj=1;jj<height;jj++) 
	// 	{
	// 	  if((abs(depthMD(ii,jj-1)-depthMD(ii,jj))>t1)&&(abs(depthMD(ii,jj-1)-depthMD(ii,jj))<t2)&&(depthMD(ii,jj-1)!=0)&&(depthMD(ii,jj)!=0))
	// 		{
	// 			img1.at<cv::Vec3b>(jj,ii)[0]=255;
	// 			img1.at<cv::Vec3b>(jj,ii)[1]=0;
	// 			img1.at<cv::Vec3b>(jj,ii)[2]=0;

	// 		}
	// 	}
	// }
	
    // for(int y=0; y<cv_ptr->image.rows; y++){
    //   float *data = cv_ptr->image.ptr<float>(height);
    //   cv::Vec3b *rgb = img1.ptr<cv::Vec3b>(height);//img1
    //   for(int x=0; x<cv_ptr->image.cols ; x++){
    //       if ( (abs(data[x]-data[x-1])>=t1) && (abs(data[x]-data[x-1])<=t2) && (data[x]!=0)&& (data[x-1]!=0) ) //mark stairs blue points
    //         {
    //             rgb[x][0]=255;
    //             rgb[x][1]=0;
    //             rgb[x][2]=0;

    //         }
         
    //   }
    // }
    for(int y=0;y < cv_ptr2->image.rows;y++)//480-> 376 height
    {
        for(int x=0;x < cv_ptr2->image.cols;x++)//640-> 672 width
        {
            if( std::isnan(cv_ptr2->image.at<float>(y,x)))//process nan problem
            {
                Arr[x+y*cv_ptr2->image.cols]=0;
                img1.at<cv::Vec3b>(y,x)[0]=255;
            }
            else if(cv_ptr2->image.at<float>(y,x)>20)
                Arr[x+y*cv_ptr2->image.cols]=20; //process infinite problem
            else if(cv_ptr2->image.at<float>(y,x)<0)
            {
                Arr[x+y*cv_ptr2->image.cols]=0;//process negative problem
                img1.at<cv::Vec3b>(y,x)[0]=255;
            }
            else if (! std::isfinite(cv_ptr2->image.at<float>(y,x)))
            {
                Arr[x+y*cv_ptr2->image.cols]=0;//process infinite
                img1.at<cv::Vec3b>(y,x)[0]=255;
            }
            else
                Arr[x+y*cv_ptr2->image.cols]=cv_ptr2->image.at<float>(y,x);


        }
    }

}

void StairDetection::Depth_Arr()
{
	for(int i=0;i<width*height;i++)
	{
		depth[i]=Arr[i]*100;
	}
}


int StairDetection::depthMD(int x,int y)
{
    int idx=0;
    
    if(x>row)
	x=row;
    if(x<0)
	x=0;
    if(y>col)
	y=col;
    if(y<0)
	y=0;
    
    idx = depth[x+y*width];  //Calculate the coordinates of the location

    if( std::isnan(idx))//process nan problem
       idx=0;
    else if(idx>2000)
       idx=2000; //process infinite problem
    else if(idx<0)
       idx=0;//process negative problem
    else if (! std::isfinite(idx))
       idx=0;//process infinite
    else
       idx=idx;


	
    return idx;

}

void StairDetection::edge_find(cv::Mat &input,int *midx,int *midy,int *di,int *dj,int *dx,int *dsw)
{
    int ddown=0,rright=0,score=0,confidence=300;
    std::vector<int> leftdown;
    std::vector<int> righttop;
	ros::param::get(NameSpace+"/t4",t4);
	ros::param::get(NameSpace+"/down",ddown);
	ros::param::get(NameSpace+"/right",rright);
    ros::param::get(NameSpace+"/confidence",confidence);
    ros::param::get(NameSpace+"/ROI_leftdown",leftdown);
    ros::param::get(NameSpace+"/ROI_righttop",righttop);
//    std::cout<<"leftdown="<<leftdown[0]<<","<<leftdown[1]<<"\n";
//    std::cout<<"righttop="<<righttop[0]<<","<<righttop[1]<<"\n";
	int mid_x=0,mid_y=0,x=0,y=0,sw=0,i=0,j=0,count=0;
	*midx=mid_x;
	*midy=mid_y;
    *di=i,*dj=j,*dx=x,*dsw=sw;
	cv::Mat ptr2;
	input.copyTo(ptr2);
//     for(int ii=0;ii<width;ii++)
//     {
//        for(int jj=1;jj<height;jj++)
//        {
//            if(ptr2.at<cv::Vec3b>(jj,ii)[0]==0)
//            {
//                dst3.at<cv::Vec3b>(jj,ii)[0]=255;
//                dst3.at<cv::Vec3b>(jj,ii)[1]=0;
//                dst3.at<cv::Vec3b>(jj,ii)[2]=0;

//            }
//        }
//     }
    for (j=leftdown[1];j>=righttop[1];j--)//reading from bottom to top , from left to right
	{
        for(i=leftdown[0];i<righttop[0];i+=2)
		{
			int right=0;
			count=0;x=i;sw=j;//start ~
            while(x<righttop[0])
			{
				int up=0,down=0;
				while(ptr2.at<uchar>(sw,x)==0)//search the feature point in a column 
				{
                    if(img1.at<cv::Vec3b>(sw,x)[0]==255)//Do not search the nan region
                    {
                        right++;
                        break;
                    }

					up++;
					if(ptr2.at<uchar>(sw+up,x)==255)//search up
					{
						sw=sw+up;
						right=0;
						break;
					}
					down--;
					if(ptr2.at<uchar>(sw+down,x)==255)//search down
					{
						sw=sw+down;
						right=0;
						break;
					}
					if(down<-ddown)//search fail,go to next column...
					{
						right++;
						break;
					}

				}
				if(right>=rright) // 5 time fails , search end. 
					break;
				if(x>row-5)// search end. 
					break;
				count++;

				x++;

			}
			if(count>t4)
			{
				mid_x=(x+i)/2;
				mid_y=(j+sw)/2;
				// std::cout<<"x="<<x<<"\n";
				// std::cout<<"i="<<i<<"\n";
				// std::cout<<"j="<<j<<"\n";
				// std::cout<<"sw="<<sw<<"\n";
//				std::cout<<"mid_x="<<mid_x<<"\n";
//				std::cout<<"mid_y="<<mid_y<<"\n";
				*midx=mid_x;
				*midy=mid_y;
                *di=i,*dj=j,*dx=x,*dsw=sw;
                cv::line( dst3, cv::Point(i,j),cv::Point(x,sw), cv::Scalar(0,254,0), 2, CV_AA);

				break;//inside for
			}
		}
		if(count>t4)
			break; //outside for

	}

    for (j=leftdown[1];j>=righttop[1];j--)//reading from bottom to top , from left to right
    {
        for(i=leftdown[0];i<righttop[0];i++)
        {
            if(ptr2.at<uchar>(j,i)==255 && dst3.at<cv::Vec3b>(j,i)[1]==254)//calculate the points overlap with line.
                score++;
        }
    }
    std::cout<<"score="<<score<<"\n";
    if(score<confidence)//If the overlap rate is too low ,then throw away this result.
    {

        *midx=0;
        *midy=0;
        *di=0,*dj=0,*dx=0,*dsw=0;
    }

}
void StairDetection::depth_find(int dc,int *mid_x2,int *mid_y2,int *midx,int *midy,int *di,int *dj,int *dx,int *dsw)
{
    /*--PS. " *XXXX " Define in mode X---*/
    int DMA_i,DMA_j;
    int mid_x=0,mid_y=0,c=0,cont=0,x=0,y=0,sw=0,i=0,j=0;
	ros::param::get(NameSpace+"/t1",t1);
	ros::param::get(NameSpace+"/t2",t2);
	ros::param::get(NameSpace+"/t3",t3);
	ros::param::get(NameSpace+"/t4",t4);
    //int ptr2[row][col] = {};

	/*---Dynamic Memory Allocation : new ---*/
    int **ptr2=NULL;
	ptr2= new int*[row];
	for(DMA_i=0; DMA_i<row; DMA_i++)  ptr2[DMA_i]=new int[col];
    for(DMA_i=0; DMA_i<row; DMA_i++)
    {
        for(DMA_j=0; DMA_j<col; DMA_j++)  ptr2[DMA_i][DMA_j]=0;
    }
    if(dc==21)	/*---For Mode 21---*/
	{

		for(int ii=0;ii<width;ii++) 
		{
			for(int jj=1;jj<height;jj++) 
			{
				if((abs(depthMD(ii,jj-1)-depthMD(ii,jj))>t1)&&(abs(depthMD(ii,jj-1)-depthMD(ii,jj))<t2)&&(depthMD(ii,jj-1)!=0)&&(depthMD(ii,jj)!=0))
				{
					c=1;
					ptr2[ii][jj]=c;
					img1.at<cv::Vec3b>(jj,ii)[0]=255;
					img1.at<cv::Vec3b>(jj,ii)[1]=0;
					img1.at<cv::Vec3b>(jj,ii)[2]=0;

				}
			}
		}

		//Process depth data
		i=0;
		j=0;
		for(j=col-5;j>=5;j-=5)//for(j=col-30;j>=35;j-=25)
		{
			for(i=5;i<=row-5;i+=5)
			{
				cont=0;x=0;
				if(ptr2[i][j-1]==1||ptr2[i][j]==1||ptr2[i][j+1]==1||ptr2[i][j-2]==1||ptr2[i][j+2]==1||ptr2[i][j+3]==1||ptr2[i][j+4]==1||ptr2[i][j+5]==1||ptr2[i][j+6]==1||ptr2[i][j+7]==1||ptr2[i][j-3]==1||ptr2[i][j-4]==1||ptr2[i][j-5]==1||ptr2[i][j-6]==1||ptr2[i][j-7]==1||ptr2[i][j-8]==1||ptr2[i][j-9]==1||ptr2[i][j-10]==1||ptr2[i][j-11]==1||ptr2[i][j-12]==1||ptr2[i][j+8]==1||ptr2[i][j+9]==1||ptr2[i][j+10]==1||ptr2[i][j+11]==1||ptr2[i][j+12]==1)
				{
					cont++; x=i;sw=j;
					while(i<row-5)
					{
						if(ptr2[i+1][sw-1]==1||ptr2[i+1][sw]==1||ptr2[i+1][sw+1]==1||ptr2[i+1][sw-2]==1||ptr2[i+1][sw+2]==1||ptr2[i+1][sw+3]==1||ptr2[i+1][sw+4]==1||ptr2[i+1][sw+5]==1||ptr2[i+1][sw+6]==1||ptr2[i+1][sw+7]==1||ptr2[i+1][sw-3]==1||ptr2[i+1][sw-4]==1||ptr2[i+1][sw-5]==1||ptr2[i+1][sw-6]==1||ptr2[i+1][sw-7]==1||ptr2[i+1][sw-8]==1||ptr2[i+1][sw-9]==1||ptr2[i+1][sw-10]==1||ptr2[i+1][sw-11]==1||ptr2[i+1][sw-12]==1||ptr2[i+1][sw+8]==1||ptr2[i+1][sw+9]==1||ptr2[i+1][sw+10]==1||ptr2[i+1][sw+11]==1||ptr2[i+1][sw+12]==1)
						{
							if(ptr2[i+1][sw-1]==1) sw=sw-1;
							else if(ptr2[i+1][sw]==1) sw=sw;
							else if(ptr2[i+1][sw+1]==1) sw=sw+1;
							else if(ptr2[i+1][sw+2]==1) sw=sw+2;
							else if(ptr2[i+1][sw-2]==1) sw=sw-2;
							else if(ptr2[i+1][sw+3]==1) sw=sw+3;
							else if(ptr2[i+1][sw-3]==1) sw=sw-3;
							else if(ptr2[i+1][sw+4]==1) sw=sw+4;
							else if(ptr2[i+1][sw-4]==1) sw=sw-4;
							else if(ptr2[i+1][sw+5]==1) sw=sw+5;
							else if(ptr2[i+1][sw-5]==1) sw=sw-5;
							else if(ptr2[i+1][sw+6]==1) sw=sw+6;
							else if(ptr2[i+1][sw-6]==1) sw=sw-6;
							else if(ptr2[i+1][sw+7]==1) sw=sw+7;
							else if(ptr2[i+1][sw-7]==1) sw=sw-7;
							else if(ptr2[i+1][sw+8]==1) sw=sw+8;
							else if(ptr2[i+1][sw-8]==1) sw=sw-8;
							else if(ptr2[i+1][sw+9]==1) sw=sw+9;
							else if(ptr2[i+1][sw-9]==1) sw=sw-9;
							else if(ptr2[i+1][sw+10]==1) sw=sw+10;
							else if(ptr2[i+1][sw-10]==1) sw=sw-10;
							else if(ptr2[i+1][sw+11]==1) sw=sw+11;
							else if(ptr2[i+1][sw-11]==1) sw=sw-11;
							else if(ptr2[i+1][sw+12]==1) sw=sw+12;
							else if(ptr2[i+1][sw-12]==1) sw=sw-12;
							cont++; i++;
						}
						else if(ptr2[i+2][sw-1]==1||ptr2[i+2][sw]==1||ptr2[i+2][sw+1]==1||ptr2[i+2][sw-2]==1||ptr2[i+2][sw+2]==1||ptr2[i+2][sw+3]==1||ptr2[i+2][sw+4]==1||ptr2[i+2][sw+5]==1||ptr2[i+2][sw+6]==1||ptr2[i+2][sw+7]==1||ptr2[i+2][sw-3]==1||ptr2[i+2][sw-4]==1||ptr2[i+2][sw-5]==1||ptr2[i+2][sw-6]==1||ptr2[i+2][sw-7]==1||ptr2[i+2][sw-8]==1||ptr2[i+2][sw-9]==1||ptr2[i+2][sw-10]==1||ptr2[i+2][sw-11]==1||ptr2[i+2][sw-12]==1||ptr2[i+2][sw+8]==1||ptr2[i+2][sw+9]==1||ptr2[i+2][sw+10]==1||ptr2[i+2][sw+11]==1||ptr2[i+2][sw+12]==1)
						{
							if(ptr2[i+2][sw-1]==1) sw=sw-1;
							else if(ptr2[i+2][sw]==1) sw=sw;
							else if(ptr2[i+2][sw+1]==1) sw=sw+1;
							else if(ptr2[i+2][sw+2]==1) sw=sw+2;
							else if(ptr2[i+2][sw-2]==1) sw=sw-2;
							else if(ptr2[i+2][sw+3]==1) sw=sw+3;
							else if(ptr2[i+2][sw-3]==1) sw=sw-3;
							else if(ptr2[i+2][sw+4]==1) sw=sw+4;
							else if(ptr2[i+2][sw-4]==1) sw=sw-4;
							else if(ptr2[i+2][sw+5]==1) sw=sw+5;
							else if(ptr2[i+2][sw-5]==1) sw=sw-5;
							else if(ptr2[i+2][sw+6]==1) sw=sw+6;
							else if(ptr2[i+2][sw-6]==1) sw=sw-6;
							else if(ptr2[i+2][sw+7]==1) sw=sw+7;
							else if(ptr2[i+2][sw-7]==1) sw=sw-7;
							else if(ptr2[i+2][sw+8]==1) sw=sw+8;
							else if(ptr2[i+2][sw-8]==1) sw=sw-8;
							else if(ptr2[i+2][sw+9]==1) sw=sw+9;
							else if(ptr2[i+2][sw-9]==1) sw=sw-9;
							else if(ptr2[i+2][sw+10]==1) sw=sw+10;
							else if(ptr2[i+2][sw-10]==1) sw=sw-10;
							else if(ptr2[i+2][sw+11]==1) sw=sw+11;
							else if(ptr2[i+2][sw-11]==1) sw=sw-11;
							else if(ptr2[i+2][sw+12]==1) sw=sw+12;
							else if(ptr2[i+2][sw-12]==1) sw=sw-12;
							cont+=2; i+=2;
						}
						else if(ptr2[i+3][sw-1]==1||ptr2[i+3][sw]==1||ptr2[i+3][sw+1]==1||ptr2[i+3][sw-2]==1||ptr2[i+3][sw+2]==1||ptr2[i+3][sw+3]==1||ptr2[i+3][sw+4]==1||ptr2[i+3][sw+5]==1||ptr2[i+3][sw+6]==1||ptr2[i+3][sw+7]==1||ptr2[i+3][sw-3]==1||ptr2[i+3][sw-4]==1||ptr2[i+3][sw-5]==1||ptr2[i+3][sw-6]==1||ptr2[i+3][sw-7]==1||ptr2[i+3][sw-8]==1||ptr2[i+3][sw-9]==1||ptr2[i+3][sw-10]==1||ptr2[i+3][sw-11]==1||ptr2[i+3][sw-12]==1||ptr2[i+3][sw+8]==1||ptr2[i+3][sw+9]==1||ptr2[i+3][sw+10]==1||ptr2[i+3][sw+11]==1||ptr2[i+3][sw+12]==1)
						{
							if(ptr2[i+3][sw-1]==1) sw=sw-1;
							else if(ptr2[i+3][sw]==1) sw=sw;
							else if(ptr2[i+3][sw+1]==1) sw=sw+1;
							else if(ptr2[i+3][sw+2]==1) sw=sw+2;
							else if(ptr2[i+3][sw-2]==1) sw=sw-2;
							else if(ptr2[i+3][sw+3]==1) sw=sw+3;
							else if(ptr2[i+3][sw-3]==1) sw=sw-3;
							else if(ptr2[i+3][sw+4]==1) sw=sw+4;
							else if(ptr2[i+3][sw-4]==1) sw=sw-4;
							else if(ptr2[i+3][sw+5]==1) sw=sw+5;
							else if(ptr2[i+3][sw-5]==1) sw=sw-5;
							else if(ptr2[i+3][sw+6]==1) sw=sw+6;
							else if(ptr2[i+3][sw-6]==1) sw=sw-6;
							else if(ptr2[i+3][sw+7]==1) sw=sw+7;
							else if(ptr2[i+3][sw-7]==1) sw=sw-7;
							else if(ptr2[i+3][sw+8]==1) sw=sw+8;
							else if(ptr2[i+3][sw-8]==1) sw=sw-8;
							else if(ptr2[i+3][sw+9]==1) sw=sw+9;
							else if(ptr2[i+3][sw-9]==1) sw=sw-9;
							else if(ptr2[i+3][sw+10]==1) sw=sw+10;
							else if(ptr2[i+3][sw-10]==1) sw=sw-10;
							else if(ptr2[i+3][sw+11]==1) sw=sw+11;
							else if(ptr2[i+3][sw-11]==1) sw=sw-11;
							else if(ptr2[i+3][sw+12]==1) sw=sw+12;
							else if(ptr2[i+3][sw-12]==1) sw=sw-12;
							cont+=3; i+=3;
						}
						else if(ptr2[i+4][sw-1]==1||ptr2[i+4][sw]==1||ptr2[i+4][sw+1]==1||ptr2[i+4][sw-2]==1||ptr2[i+4][sw+2]==1||ptr2[i+4][sw+3]==1||ptr2[i+4][sw+4]==1||ptr2[i+4][sw+5]==1||ptr2[i+4][sw+6]==1||ptr2[i+4][sw+7]==1||ptr2[i+4][sw-3]==1||ptr2[i+4][sw-4]==1||ptr2[i+4][sw-5]==1||ptr2[i+4][sw-6]==1||ptr2[i+4][sw-7]==1||ptr2[i+4][sw-8]==1||ptr2[i+4][sw-9]==1||ptr2[i+4][sw-10]==1||ptr2[i+4][sw-11]==1||ptr2[i+4][sw-12]==1||ptr2[i+4][sw+8]==1||ptr2[i+4][sw+9]==1||ptr2[i+4][sw+10]==1||ptr2[i+4][sw+11]==1||ptr2[i+4][sw+12]==1)
						{
							if(ptr2[i+4][sw-1]==1) sw=sw-1;
							else if(ptr2[i+4][sw]==1) sw=sw;
							else if(ptr2[i+4][sw+1]==1) sw=sw+1;
							else if(ptr2[i+4][sw+2]==1) sw=sw+2;
							else if(ptr2[i+4][sw-2]==1) sw=sw-2;
							else if(ptr2[i+4][sw+3]==1) sw=sw+3;
							else if(ptr2[i+4][sw-3]==1) sw=sw-3;
							else if(ptr2[i+4][sw+4]==1) sw=sw+4;
							else if(ptr2[i+4][sw-4]==1) sw=sw-4;
							else if(ptr2[i+4][sw+5]==1) sw=sw+5;
							else if(ptr2[i+4][sw-5]==1) sw=sw-5;
							else if(ptr2[i+4][sw+6]==1) sw=sw+6;
							else if(ptr2[i+4][sw-6]==1) sw=sw-6;
							else if(ptr2[i+4][sw+7]==1) sw=sw+7;
							else if(ptr2[i+4][sw-7]==1) sw=sw-7;
							else if(ptr2[i+4][sw+8]==1) sw=sw+8;
							else if(ptr2[i+4][sw-8]==1) sw=sw-8;
							else if(ptr2[i+4][sw+9]==1) sw=sw+9;
							else if(ptr2[i+4][sw-9]==1) sw=sw-9;
							else if(ptr2[i+4][sw+10]==1) sw=sw+10;
							else if(ptr2[i+4][sw-10]==1) sw=sw-10;
							else if(ptr2[i+4][sw+11]==1) sw=sw+11;
							else if(ptr2[i+4][sw-11]==1) sw=sw-11;
							else if(ptr2[i+4][sw+12]==1) sw=sw+12;
							else if(ptr2[i+4][sw-12]==1) sw=sw-12;
							cont+=4; i+=4;
						}
						else	break ;//out while
					}
				}
				if(cont>t3)
				{
					mid_x=(x+i)/2;
					mid_y=(j+sw)/2;
//					 std::cout<<"x="<<x<<"\n";
//					 std::cout<<"i="<<i<<"\n";
//					 std::cout<<"j="<<j<<"\n";
//					 std::cout<<"sw="<<sw<<"\n";
//					 std::cout<<"mid_x="<<mid_x<<"\n";
//					 std::cout<<"mid_y="<<mid_y<<"\n";

					*midx=mid_x;
					*midy=mid_y;

					break;//inside for
				}
			}
			if(cont>t3)
			{
				// std::cout<<"midff\n";
				
				break; //outside for
			}
		}
		////////////////////////////////NO2////////////////////////////
		// i=0;
		// for(j=mid_y-30;j>=35;j-=25)//for(j=mid_y-30;j>=35;j-=25)
		// {
		// 	for(i=5;i<=row-5;i+=5)
		// 	{
		// 		cont=0;x=0;sw=0;
		// 		if(ptr2[i][j-1]==1||ptr2[i][j]==1||ptr2[i][j+1]==1||ptr2[i][j-2]==1||ptr2[i][j+2]==1||ptr2[i][j+3]==1||ptr2[i][j+4]==1||ptr2[i][j+5]==1||ptr2[i][j+6]==1||ptr2[i][j+7]==1||ptr2[i][j-3]==1||ptr2[i][j-4]==1||ptr2[i][j-5]==1||ptr2[i][j-6]==1||ptr2[i][j-7]==1||ptr2[i][j-8]==1||ptr2[i][j-9]==1||ptr2[i][j-10]==1||ptr2[i][j-11]==1||ptr2[i][j-12]==1||ptr2[i][j+8]==1||ptr2[i][j+9]==1||ptr2[i][j+10]==1||ptr2[i][j+11]==1||ptr2[i][j+12]==1)
		// 		{
		// 			cont++; x=i;sw=j;
		// 			while(i<row-5)
		// 			{
		// 				if(ptr2[i+1][sw-1]==1||ptr2[i+1][sw]==1||ptr2[i+1][sw+1]==1||ptr2[i+1][sw-2]==1||ptr2[i+1][sw+2]==1||ptr2[i+1][sw+3]==1||ptr2[i+1][sw+4]==1||ptr2[i+1][sw+5]==1||ptr2[i+1][sw+6]==1||ptr2[i+1][sw+7]==1||ptr2[i+1][sw-3]==1||ptr2[i+1][sw-4]==1||ptr2[i+1][sw-5]==1||ptr2[i+1][sw-6]==1||ptr2[i+1][sw-7]==1||ptr2[i+1][sw-8]==1||ptr2[i+1][sw-9]==1||ptr2[i+1][sw-10]==1||ptr2[i+1][sw-11]==1||ptr2[i+1][sw-12]==1||ptr2[i+1][sw+8]==1||ptr2[i+1][sw+9]==1||ptr2[i+1][sw+10]==1||ptr2[i+1][sw+11]==1||ptr2[i+1][sw+12]==1)
		// 				{
		// 					if(ptr2[i+1][sw-1]==1) sw=sw-1;
		// 					else if(ptr2[i+1][sw]==1) sw=sw;
		// 					else if(ptr2[i+1][sw+1]==1) sw=sw+1;
		// 					else if(ptr2[i+1][sw+2]==1) sw=sw+2;
		// 					else if(ptr2[i+1][sw-2]==1) sw=sw-2;
		// 					else if(ptr2[i+1][sw+3]==1) sw=sw+3;
		// 					else if(ptr2[i+1][sw-3]==1) sw=sw-3;
		// 					else if(ptr2[i+1][sw+4]==1) sw=sw+4;
		// 					else if(ptr2[i+1][sw-4]==1) sw=sw-4;
		// 					else if(ptr2[i+1][sw+5]==1) sw=sw+5;
		// 					else if(ptr2[i+1][sw-5]==1) sw=sw-5;
		// 					else if(ptr2[i+1][sw+6]==1) sw=sw+6;
		// 					else if(ptr2[i+1][sw-6]==1) sw=sw-6;
		// 					else if(ptr2[i+1][sw+7]==1) sw=sw+7;
		// 					else if(ptr2[i+1][sw-7]==1) sw=sw-7;
		// 					else if(ptr2[i+1][sw+8]==1) sw=sw+8;
		// 					else if(ptr2[i+1][sw-8]==1) sw=sw-8;
		// 					else if(ptr2[i+1][sw+9]==1) sw=sw+9;
		// 					else if(ptr2[i+1][sw-9]==1) sw=sw-9;
		// 					else if(ptr2[i+1][sw+10]==1) sw=sw+10;
		// 					else if(ptr2[i+1][sw-10]==1) sw=sw-10;
		// 					else if(ptr2[i+1][sw+11]==1) sw=sw+11;
		// 					else if(ptr2[i+1][sw-11]==1) sw=sw-11;
		// 					else if(ptr2[i+1][sw+12]==1) sw=sw+12;
		// 					else if(ptr2[i+1][sw-12]==1) sw=sw-12;
		// 					cont++; i++;
		// 				}
		// 				else if(ptr2[i+2][sw-1]==1||ptr2[i+2][sw]==1||ptr2[i+2][sw+1]==1||ptr2[i+2][sw-2]==1||ptr2[i+2][sw+2]==1||ptr2[i+2][sw+3]==1||ptr2[i+2][sw+4]==1||ptr2[i+2][sw+5]==1||ptr2[i+2][sw+6]==1||ptr2[i+2][sw+7]==1||ptr2[i+2][sw-3]==1||ptr2[i+2][sw-4]==1||ptr2[i+2][sw-5]==1||ptr2[i+2][sw-6]==1||ptr2[i+2][sw-7]==1||ptr2[i+2][sw-8]==1||ptr2[i+2][sw-9]==1||ptr2[i+2][sw-10]==1||ptr2[i+2][sw-11]==1||ptr2[i+2][sw-12]==1||ptr2[i+2][sw+8]==1||ptr2[i+2][sw+9]==1||ptr2[i+2][sw+10]==1||ptr2[i+2][sw+11]==1||ptr2[i+2][sw+12]==1)
		// 				{
		// 					if(ptr2[i+2][sw-1]==1) sw=sw-1;
		// 					else if(ptr2[i+2][sw]==1) sw=sw;
		// 					else if(ptr2[i+2][sw+1]==1) sw=sw+1;
		// 					else if(ptr2[i+2][sw+2]==1) sw=sw+2;
		// 					else if(ptr2[i+2][sw-2]==1) sw=sw-2;
		// 					else if(ptr2[i+2][sw+3]==1) sw=sw+3;
		// 					else if(ptr2[i+2][sw-3]==1) sw=sw-3;
		// 					else if(ptr2[i+2][sw+4]==1) sw=sw+4;
		// 					else if(ptr2[i+2][sw-4]==1) sw=sw-4;
		// 					else if(ptr2[i+2][sw+5]==1) sw=sw+5;
		// 					else if(ptr2[i+2][sw-5]==1) sw=sw-5;
		// 					else if(ptr2[i+2][sw+6]==1) sw=sw+6;
		// 					else if(ptr2[i+2][sw-6]==1) sw=sw-6;
		// 					else if(ptr2[i+2][sw+7]==1) sw=sw+7;
		// 					else if(ptr2[i+2][sw-7]==1) sw=sw-7;
		// 					else if(ptr2[i+2][sw+8]==1) sw=sw+8;
		// 					else if(ptr2[i+2][sw-8]==1) sw=sw-8;
		// 					else if(ptr2[i+2][sw+9]==1) sw=sw+9;
		// 					else if(ptr2[i+2][sw-9]==1) sw=sw-9;
		// 					else if(ptr2[i+2][sw+10]==1) sw=sw+10;
		// 					else if(ptr2[i+2][sw-10]==1) sw=sw-10;
		// 					else if(ptr2[i+2][sw+11]==1) sw=sw+11;
		// 					else if(ptr2[i+2][sw-11]==1) sw=sw-11;
		// 					else if(ptr2[i+2][sw+12]==1) sw=sw+12;
		// 					else if(ptr2[i+2][sw-12]==1) sw=sw-12;
		// 					cont+=2; i+=2;
		// 				}
		// 				else if(ptr2[i+3][sw-1]==1||ptr2[i+3][sw]==1||ptr2[i+3][sw+1]==1||ptr2[i+3][sw-2]==1||ptr2[i+3][sw+2]==1||ptr2[i+3][sw+3]==1||ptr2[i+3][sw+4]==1||ptr2[i+3][sw+5]==1||ptr2[i+3][sw+6]==1||ptr2[i+3][sw+7]==1||ptr2[i+3][sw-3]==1||ptr2[i+3][sw-4]==1||ptr2[i+3][sw-5]==1||ptr2[i+3][sw-6]==1||ptr2[i+3][sw-7]==1||ptr2[i+3][sw-8]==1||ptr2[i+3][sw-9]==1||ptr2[i+3][sw-10]==1||ptr2[i+3][sw-11]==1||ptr2[i+3][sw-12]==1||ptr2[i+3][sw+8]==1||ptr2[i+3][sw+9]==1||ptr2[i+3][sw+10]==1||ptr2[i+3][sw+11]==1||ptr2[i+3][sw+12]==1)
		// 				{
		// 					if(ptr2[i+3][sw-1]==1) sw=sw-1;
		// 					else if(ptr2[i+3][sw]==1) sw=sw;
		// 					else if(ptr2[i+3][sw+1]==1) sw=sw+1;
		// 					else if(ptr2[i+3][sw+2]==1) sw=sw+2;
		// 					else if(ptr2[i+3][sw-2]==1) sw=sw-2;
		// 					else if(ptr2[i+3][sw+3]==1) sw=sw+3;
		// 					else if(ptr2[i+3][sw-3]==1) sw=sw-3;
		// 					else if(ptr2[i+3][sw+4]==1) sw=sw+4;
		// 					else if(ptr2[i+3][sw-4]==1) sw=sw-4;
		// 					else if(ptr2[i+3][sw+5]==1) sw=sw+5;
		// 					else if(ptr2[i+3][sw-5]==1) sw=sw-5;
		// 					else if(ptr2[i+3][sw+6]==1) sw=sw+6;
		// 					else if(ptr2[i+3][sw-6]==1) sw=sw-6;
		// 					else if(ptr2[i+3][sw+7]==1) sw=sw+7;
		// 					else if(ptr2[i+3][sw-7]==1) sw=sw-7;
		// 					else if(ptr2[i+3][sw+8]==1) sw=sw+8;
		// 					else if(ptr2[i+3][sw-8]==1) sw=sw-8;
		// 					else if(ptr2[i+3][sw+9]==1) sw=sw+9;
		// 					else if(ptr2[i+3][sw-9]==1) sw=sw-9;
		// 					else if(ptr2[i+3][sw+10]==1) sw=sw+10;
		// 					else if(ptr2[i+3][sw-10]==1) sw=sw-10;
		// 					else if(ptr2[i+3][sw+11]==1) sw=sw+11;
		// 					else if(ptr2[i+3][sw-11]==1) sw=sw-11;
		// 					else if(ptr2[i+3][sw+12]==1) sw=sw+12;
		// 					else if(ptr2[i+3][sw-12]==1) sw=sw-12;
		// 					cont+=3; i+=3;
		// 				}
		// 				else if(ptr2[i+4][sw-1]==1||ptr2[i+4][sw]==1||ptr2[i+4][sw+1]==1||ptr2[i+4][sw-2]==1||ptr2[i+4][sw+2]==1||ptr2[i+4][sw+3]==1||ptr2[i+4][sw+4]==1||ptr2[i+4][sw+5]==1||ptr2[i+4][sw+6]==1||ptr2[i+4][sw+7]==1||ptr2[i+4][sw-3]==1||ptr2[i+4][sw-4]==1||ptr2[i+4][sw-5]==1||ptr2[i+4][sw-6]==1||ptr2[i+4][sw-7]==1||ptr2[i+4][sw-8]==1||ptr2[i+4][sw-9]==1||ptr2[i+4][sw-10]==1||ptr2[i+4][sw-11]==1||ptr2[i+4][sw-12]==1||ptr2[i+4][sw+8]==1||ptr2[i+4][sw+9]==1||ptr2[i+4][sw+10]==1||ptr2[i+4][sw+11]==1||ptr2[i+4][sw+12]==1)
		// 				{
		// 					if(ptr2[i+4][sw-1]==1) sw=sw-1;
		// 					else if(ptr2[i+4][sw]==1) sw=sw;
		// 					else if(ptr2[i+4][sw+1]==1) sw=sw+1;
		// 					else if(ptr2[i+4][sw+2]==1) sw=sw+2;
		// 					else if(ptr2[i+4][sw-2]==1) sw=sw-2;
		// 					else if(ptr2[i+4][sw+3]==1) sw=sw+3;
		// 					else if(ptr2[i+4][sw-3]==1) sw=sw-3;
		// 					else if(ptr2[i+4][sw+4]==1) sw=sw+4;
		// 					else if(ptr2[i+4][sw-4]==1) sw=sw-4;
		// 					else if(ptr2[i+4][sw+5]==1) sw=sw+5;
		// 					else if(ptr2[i+4][sw-5]==1) sw=sw-5;
		// 					else if(ptr2[i+4][sw+6]==1) sw=sw+6;
		// 					else if(ptr2[i+4][sw-6]==1) sw=sw-6;
		// 					else if(ptr2[i+4][sw+7]==1) sw=sw+7;
		// 					else if(ptr2[i+4][sw-7]==1) sw=sw-7;
		// 					else if(ptr2[i+4][sw+8]==1) sw=sw+8;
		// 					else if(ptr2[i+4][sw-8]==1) sw=sw-8;
		// 					else if(ptr2[i+4][sw+9]==1) sw=sw+9;
		// 					else if(ptr2[i+4][sw-9]==1) sw=sw-9;
		// 					else if(ptr2[i+4][sw+10]==1) sw=sw+10;
		// 					else if(ptr2[i+4][sw-10]==1) sw=sw-10;
		// 					else if(ptr2[i+4][sw+11]==1) sw=sw+11;
		// 					else if(ptr2[i+4][sw-11]==1) sw=sw-11;
		// 					else if(ptr2[i+4][sw+12]==1) sw=sw+12;
		// 					else if(ptr2[i+4][sw-12]==1) sw=sw-12;
		// 					cont+=4; i+=4;
		// 				}
		// 				else	break ;//out while
		// 			}
		// 		}
		// 		// std::cout<<"msasid\n";
		// 		if(cont>t3)
		// 		{
		// 			*di=i;
		// 			*dj=j;
		// 			*dx=x;
		// 			*dsw=sw;

		// 			*mid_x2=(x+i)/2;
		// 			*mid_y2=(j+sw)/2;
		// 			// std::cout<<"midff555\n";
		// 			break;//inside for
		// 		}
			// }
			// if(cont>t3)
			// {
			// 	// std::cout<<"midff5454\n";
			// 	break; //outside for
			// }
		// }

	}//else if(dc==21) end

    if(dc==22)	/*-- Mode 21 revised version---*/
	{

		for(int ii=0;ii<width;ii++) 
		{
			for(int jj=1;jj<height;jj++) 
			{
				if((abs(depthMD(ii,jj-1)-depthMD(ii,jj))>t1)&&(abs(depthMD(ii,jj-1)-depthMD(ii,jj))<t3)&&(depthMD(ii,jj-1)!=0)&&(depthMD(ii,jj)!=0))
				{
					c=1;
					ptr2[ii][jj]=c;
					img1.at<cv::Vec3b>(jj,ii)[0]=255;
					img1.at<cv::Vec3b>(jj,ii)[1]=0;
					img1.at<cv::Vec3b>(jj,ii)[2]=0;


				}
			}
		}

		//Process depth data
		i=0,j=0;
		for(j=col-15;j>=15;j-=15)//for(j=col-30;j>=35;j-=25)
		{
			for(i=5;i<=row-5;i+=5)
			{
				cont=0;x=i;sw=j;//start ~
				int right=0;
				while(x<row-5) //find the stair edge
				{
					int up=0,down=0;
					while(!ptr2[x][sw])//search the feature point in a column 
					{	
						up++;
						if(ptr2[x][sw+up]==1)//search up
						{
							sw=sw+up;
							right=0;
							//std::cout<<"searching column success\n";
							//std::cout<<"sw="<<sw<<"\n";
							break;
						}
						down--;
						if(ptr2[x][sw+down]==1)//search down
						{
							sw=sw+down;
							right=0;
							//std::cout<<"searching column success\n";
							//std::cout<<"sw="<<sw<<"\n";
							break;
						}
						if(down<-5)//search fail,go to next column...
						{
							right++;
							//std::cout<<"searching column fail\n";
							break;
						}
						
					}
					if(right>=5) // 5 time fails , search end. 
						break;
					if(x>row-5)// search end. 
						break;
					cont++;
					x++;
					//std::cout<<"feature"<<x<<","<<sw<<"="<<ptr2[x][sw]<<"\n";
					// if(cont>t3)// search end.
					// 	break;
					//std::cout<<"right="<<right<<"\n";
					//std::cout<<"count="<<cont<<"\n";
				}
				if(cont>t4)
				{
                     mid_x=(x+i)/2;
                     mid_y=(j+sw)/2;
					//  std::cout<<"x="<<x<<"\n";
					//  std::cout<<"i="<<i<<"\n";
					//  std::cout<<"j="<<j<<"\n";
					//  std::cout<<"sw="<<sw<<"\n";
					//  std::cout<<"mid_x="<<mid_x<<"\n";
					//  std::cout<<"mid_y="<<mid_y<<"\n";

					*midx=mid_x;
					*midy=mid_y;
					cv::line( img1, cv::Point(i,j),cv::Point(x,sw), cv::Scalar(0,255,255), 1, CV_AA);

					break;//inside for
				}
			}
			if(cont>t4)
			{
				// std::cout<<"midff\n";
				
				break; //outside for
			}
		}
	}
	
    else if(dc==4)	/*---For Mode 4---*/
	{
		for(i=0;i<width;i++) {
			for(j=1;j<height;j++) {
				if((abs(depthMD(i,j-1)-depthMD(i,j))>t1)&&(depthMD(i,j-1)!=0)&&(depthMD(i,j)!=0))
				{
					c=1;
					ptr2[i][j]=c;
				}
			}
		}

		i=0;
		j=0;
		for(j=2;j<=height-2;j+=5)
		{
			for(i=0;i<=width-3;i+=3)
			{
				cont=0;x=0;sw=0;
				if(ptr2[i][j-1]==1||ptr2[i][j]==1||ptr2[i][j+1]==1||ptr2[i][j-2]==1||ptr2[i][j+2]==1)
				{
					cont++; x=i;sw=j;
					while(i<637)
					{
						if(ptr2[i+1][sw-1]==1||ptr2[i+1][sw]==1||ptr2[i+1][sw+1]==1||ptr2[i+1][sw-2]==1||ptr2[i+1][sw+2]==1)
						{
						if(ptr2[i+1][sw-1]==1) sw=sw-1;
						else if(ptr2[i+1][sw]==1) sw=sw;
						else if(ptr2[i+1][sw+1]==1) sw=sw+1;
						else if(ptr2[i+1][sw+2]==1) sw=sw+2;
						else if(ptr2[i+1][sw-2]==1) sw=sw-2;
						cont++; i++;
						}
						else if(ptr2[i+2][sw-1]==1||ptr2[i+2][sw]==1||ptr2[i+2][sw+1]==1||ptr2[i+2][sw-2]==1||ptr2[i+2][sw+2]==1)
						{
						if(ptr2[i+2][sw-1]==1) sw=sw-1;
						else if(ptr2[i+2][sw]==1) sw=sw;
						else if(ptr2[i+2][sw+1]==1) sw=sw+1;
						else if(ptr2[i+2][sw+2]==1) sw=sw+2;
						else if(ptr2[i+2][sw-2]==1) sw=sw-2;
						cont+=2; i+=2;
						}
						else	break ;//out while
					}
				}
				if(cont>t3)
				{
					*di=i;
					*dj=j;
					*dx=x;
					*dsw=sw;

					mid_x=(x+i)/2;
					mid_y=(j+sw)/2;

					*midx=mid_x;
					*midy=mid_y;

					break;//inside for
				}
			}

			if(cont>t3)    break; //outside for
		}

	}//	else if(dc==4) end

  
   
	/*---Dynamic Memory Allocation : delete ---*/

	for(int DMA_i=0; DMA_i<row; DMA_i++)    delete ptr2[DMA_i];
	delete ptr2;

} //stairs_find end


void StairDetection::depth_find_4(int dc,int *midx,int *midy,int *di,int *dj,int *dx,int *dsw)
{
    /*--PS. " *XXXX " Define in mode X---*/
    int DMA_i,DMA_j;
    int mid_x=0,mid_y=0,c=0,cont=0,x=0,y=0,sw=0,i=0,j=0;
	*midx=mid_x;
	*midy=mid_y;
    //int ptr2[row][col] = {};

	/*---Dynamic Memory Allocation : new ---*/
    int **ptr2=NULL;
	ptr2= new int*[row];
	for(DMA_i=0; DMA_i<row; DMA_i++)  ptr2[DMA_i]=new int[col];
    for(DMA_i=0; DMA_i<row; DMA_i++)
    {
        for(DMA_j=0; DMA_j<col; DMA_j++)  ptr2[DMA_i][DMA_j]=0;
    }


   if(dc==4)	/*---For Mode 4---*/
	{
		for(i=0;i<width;i++) {
			for(j=1;j<height;j++) {
				if((abs(depthMD(i,j-1)-depthMD(i,j))>t1)&&(depthMD(i,j-1)!=0)&&(depthMD(i,j)!=0))
				{
					c=1;
					ptr2[i][j]=c;
				}
			}
		}

		i=0;
		j=0;
		for(j=2;j<=height-3;j+=5)
		{
			for(i=0;i<=width-3;i+=3)
			{
				cont=0;x=0;sw=0;
				if(ptr2[i][j-1]==1||ptr2[i][j]==1||ptr2[i][j+1]==1||ptr2[i][j-2]==1||ptr2[i][j+2]==1)
				{
					cont++; x=i;sw=j;
					while(i<width-3)
					{
						if(ptr2[i+1][sw-1]==1||ptr2[i+1][sw]==1||ptr2[i+1][sw+1]==1||ptr2[i+1][sw-2]==1||ptr2[i+1][sw+2]==1)
						{
						if(ptr2[i+1][sw-1]==1) sw=sw-1;
						else if(ptr2[i+1][sw]==1) sw=sw;
						else if(ptr2[i+1][sw+1]==1) sw=sw+1;
						else if(ptr2[i+1][sw+2]==1) sw=sw+2;
						else if(ptr2[i+1][sw-2]==1) sw=sw-2;
						cont++; i++;
						}
						else if(ptr2[i+2][sw-1]==1||ptr2[i+2][sw]==1||ptr2[i+2][sw+1]==1||ptr2[i+2][sw-2]==1||ptr2[i+2][sw+2]==1)
						{
						if(ptr2[i+2][sw-1]==1) sw=sw-1;
						else if(ptr2[i+2][sw]==1) sw=sw;
						else if(ptr2[i+2][sw+1]==1) sw=sw+1;
						else if(ptr2[i+2][sw+2]==1) sw=sw+2;
						else if(ptr2[i+2][sw-2]==1) sw=sw-2;
						cont+=2; i+=2;
						}
						else	break ;//out while
					}
				}
				if(cont>t4)
				{
					*di=i;
					*dj=j;
					*dx=x;
					*dsw=sw;

					mid_x=(x+i)/2;
					mid_y=(j+sw)/2;

					*midx=mid_x;
					*midy=mid_y;

					break;//inside for
				}
			}

			if(cont>t4)    break; //outside for
		}

	}//	else if(dc==4) end

	/*---Dynamic Memory Allocation : delete ---*/

	for(int DMA_i=0; DMA_i<row; DMA_i++)    delete ptr2[DMA_i];
	delete ptr2;

} //stairs_find_4 end
			
void StairDetection::depth_find_5(int dc,int *midx,int *midy,int *di,int *dj,int *dx,int *dsw)
{

    

    /*--PS. " *XXXX " Define in mode X---*/
    int DMA_i,DMA_j;
    int mid_x=0,mid_y=0,c=0,cont=0,x=0,y=0,sw=0,i=0,j=0;
	// *midx=mid_x;
	// *midy=mid_y;
	ros::param::get(NameSpace+"/t1",t1);
	ros::param::get(NameSpace+"/t2",t2);
	ros::param::get(NameSpace+"/t3",t3);
	ros::param::get(NameSpace+"/t4",t4);
    //int ptr2[row][col] = {};

	/*---Dynamic Memory Allocation : new ---*/
    int **ptr2=NULL;
	ptr2= new int*[row];
	for(DMA_i=0; DMA_i<row; DMA_i++)  ptr2[DMA_i]=new int[col];
    for(DMA_i=0; DMA_i<row; DMA_i++)
    {
        for(DMA_j=0; DMA_j<col; DMA_j++)  ptr2[DMA_i][DMA_j]=0;
    }
    
    if(dc==22)	/*-- Mode 21 revised version---*/
	{

		for(int ii=0;ii<width;ii++) 
		{
			for(int jj=1;jj<height;jj++) 
			{
				if((abs(depthMD(ii,jj-1)-depthMD(ii,jj))>t1)&&(abs(depthMD(ii,jj-1)-depthMD(ii,jj))<t2)&&(depthMD(ii,jj-1)!=0)&&(depthMD(ii,jj)!=0))
				{
					c=1;
					ptr2[ii][jj]=c;

				}
			}
		}

		//Process depth data
		i=0,j=0;
		for(j=col-15;j>=15;j-=15)//for(j=col-30;j>=35;j-=25)
		{
			for(i=5;i<=row-5;i+=5)
			{
				cont=0;x=i;sw=j;//start ~
				int right=0;
				while(x<row-5) //find the stair edge
				{
					int up=0,down=0;
					while(!ptr2[x][sw])//search the feature point in a column 
					{	
						up++;
						if(ptr2[x][sw+up]==1)//search up
						{
							sw=sw+up;
							right=0;
							//std::cout<<"searching column success\n";
							//std::cout<<"sw="<<sw<<"\n";
							break;
						}
						down--;
						if(ptr2[x][sw+down]==1)//search down
						{
							sw=sw+down;
							right=0;
							//std::cout<<"searching column success\n";
							//std::cout<<"sw="<<sw<<"\n";
							break;
						}
						if(down<-5)//search fail,go to next column...
						{
							right++;
							//std::cout<<"searching column fail\n";
							break;
						}
						
					}
					if(right>=5) // 5 time fails , search end. 
						break;
					if(x>row-5)// search end. 
						break;
					cont++;
					x++;
					//std::cout<<"feature"<<x<<","<<sw<<"="<<ptr2[x][sw]<<"\n";
					// if(cont>t3)// search end.
					// 	break;
					//std::cout<<"right="<<right<<"\n";
					//std::cout<<"count="<<cont<<"\n";
				}
				if(cont>t3)
				{
					mid_x=(x+i)/2;
					mid_y=(j+sw)/2;
					 std::cout<<"x="<<x<<"\n";
					 std::cout<<"i="<<i<<"\n";
					 std::cout<<"j="<<j<<"\n";
					 std::cout<<"sw="<<sw<<"\n";
					 std::cout<<"mid_x="<<mid_x<<"\n";
					 std::cout<<"mid_y="<<mid_y<<"\n";

					*midx=mid_x;
					*midy=mid_y;
					

					break;//inside for
				}
			}
			if(cont>t3)
			{
				// std::cout<<"midff\n";
				
				break; //outside for
			}
		}
	}
	else if(dc==5)	/*---For Mode 5---*/
	{
		for(int ii=0;ii<width;ii++) 
		{
			for(int jj=1;jj<height;jj++) 
			{
				if((abs(depthMD(ii,jj-1)-depthMD(ii,jj))>t1)&&(abs(depthMD(ii,jj-1)-depthMD(ii,jj))<t2)&&(depthMD(ii,jj-1)!=0)&&(depthMD(ii,jj)!=0))
				{
					c=1;
					ptr2[ii][jj]=c;

				}
			}
		}

		i=0;
		j=0;
		for(j=col-3;j>=2;j-=5)
		{
			for(i=0;i<=row-3;i+=3)
			{
				cont=0;x=0;sw=0;
				if(ptr2[i][j-1]==1||ptr2[i][j]==1||ptr2[i][j+1]==1||ptr2[i][j-2]==1||ptr2[i][j+2]==1)
				{
					cont++; x=i;sw=j;
					while(i<row-3)
					{
						if(ptr2[i+1][sw-1]==1||ptr2[i+1][sw]==1||ptr2[i+1][sw+1]==1||ptr2[i+1][sw-2]==1||ptr2[i+1][sw+2]==1)
						{
							if(ptr2[i+1][sw-1]==1) sw=sw-1;
							else if(ptr2[i+1][sw]==1) sw=sw;
							else if(ptr2[i+1][sw+1]==1) sw=sw+1;
							else if(ptr2[i+1][sw+2]==1) sw=sw+2;
							else if(ptr2[i+1][sw-2]==1) sw=sw-2;
							cont++; i++;
						}
						else if(ptr2[i+2][sw-1]==1||ptr2[i+2][sw]==1||ptr2[i+2][sw+1]==1||ptr2[i+2][sw-2]==1||ptr2[i+2][sw+2]==1)
						{
							if(ptr2[i+2][sw-1]==1) sw=sw-1;
							else if(ptr2[i+2][sw]==1) sw=sw;
							else if(ptr2[i+2][sw+1]==1) sw=sw+1;
							else if(ptr2[i+2][sw+2]==1) sw=sw+2;
							else if(ptr2[i+2][sw-2]==1) sw=sw-2;
							cont+=2; i+=2;
						}
						else	break ;//out while
					}
				}

				if(cont>t3)
				{
					*di=i;
					*dj=j;
					*dx=x;
					*dsw=sw;

					mid_x=(x+i)/2;
					mid_y=(j+sw)/2;

					*midx=mid_x;
					*midy=mid_y;

					break;//inside for
				}
			}

			if(cont>t3)    break; //outside for

		}

	}
  
  
   
	/*---Dynamic Memory Allocation : delete ---*/

	for(int DMA_i=0; DMA_i<row; DMA_i++)    delete ptr2[DMA_i];
	delete ptr2;

} //stairs_find 5 end

void StairDetection::depth_find_7(int dc,int *midx,int *midy,int *di,int *dj,int *dx,int *dsw)
{
    /*--PS. " *XXXX " Define in mode X---*/
    int DMA_i,DMA_j;
    int mid_x=0,mid_y=0,c=0,cont=0,x=0,y=0,sw=0,i=0,j=0;
	*midx=mid_x;
	*midy=mid_y;
	ros::param::get(NameSpace+"/t1",t1);
	ros::param::get(NameSpace+"/t2",t2);
	ros::param::get(NameSpace+"/t3",t3);
	ros::param::get(NameSpace+"/t4",t4);
    //int ptr2[row][col] = {};

	/*---Dynamic Memory Allocation : new ---*/
    int **ptr2=NULL;
	ptr2= new int*[row];
	for(DMA_i=0; DMA_i<row; DMA_i++)  ptr2[DMA_i]=new int[col];
    for(DMA_i=0; DMA_i<row; DMA_i++)
    {
        for(DMA_j=0; DMA_j<col; DMA_j++)  ptr2[DMA_i][DMA_j]=0;
    }
    
    if(dc==22)	/*-- Mode 21 revised version---*/
	{

		for(int ii=0;ii<width;ii++) 
		{
			for(int jj=1;jj<height;jj++) 
			{
				if((abs(depthMD(ii,jj-1)-depthMD(ii,jj))>t1)&&(abs(depthMD(ii,jj-1)-depthMD(ii,jj))<t2)&&(depthMD(ii,jj-1)!=0)&&(depthMD(ii,jj)!=0))
				{
					c=1;
					ptr2[ii][jj]=c;

				}
			}
		}

		//Process depth data
		i=0,j=0;
		for(j=col-15;j>=15;j-=15)//for(j=col-30;j>=35;j-=25)
		{
			for(i=5;i<=row-5;i+=5)
			{
				cont=0;x=i;sw=j;//start ~
				int right=0;
				while(x<row-5) //find the stair edge
				{
					int up=0,down=0;
					while(!ptr2[x][sw])//search the feature point in a column 
					{	
						up++;
						if(ptr2[x][sw+up]==1)//search up
						{
							sw=sw+up;
							right=0;
							//std::cout<<"searching column success\n";
							//std::cout<<"sw="<<sw<<"\n";
							break;
						}
						down--;
						if(ptr2[x][sw+down]==1)//search down
						{
							sw=sw+down;
							right=0;
							//std::cout<<"searching column success\n";
							//std::cout<<"sw="<<sw<<"\n";
							break;
						}
						if(down<-5)//search fail,go to next column...
						{
							right++;
							//std::cout<<"searching column fail\n";
							break;
						}
						
					}
					if(right>=5) // 5 time fails , search end. 
						break;
					if(x>row-5)// search end. 
						break;
					cont++;
					x++;
					//std::cout<<"feature"<<x<<","<<sw<<"="<<ptr2[x][sw]<<"\n";
					// if(cont>t3)// search end.
					// 	break;
					//std::cout<<"right="<<right<<"\n";
					//std::cout<<"count="<<cont<<"\n";
				}
				if(cont>t3)
				{
					mid_x=(x+i)/2;
					mid_y=(j+sw)/2;
					 std::cout<<"x="<<x<<"\n";
					 std::cout<<"i="<<i<<"\n";
					 std::cout<<"j="<<j<<"\n";
					 std::cout<<"sw="<<sw<<"\n";
					 std::cout<<"mid_x="<<mid_x<<"\n";
					 std::cout<<"mid_y="<<mid_y<<"\n";

					*midx=mid_x;
					*midy=mid_y;
					

					break;//inside for
				}
			}
			if(cont>t3)
			{
				// std::cout<<"midff\n";
				
				break; //outside for
			}
		}
	}
	 else if(dc==7)	/*---For Mode 7---*/
	{

		for(int ii=0;ii<width;ii++) 
		{
			for(int jj=1;jj<height;jj++) 
			{
				if((abs(depthMD(ii,jj-1)-depthMD(ii,jj))>t1)&&(abs(depthMD(ii,jj-1)-depthMD(ii,jj))<t2)&&(depthMD(ii,jj-1)!=0)&&(depthMD(ii,jj)!=0))
				{
					c=1;
					ptr2[ii][jj]=c;

				}
			}
		}

		i=0;
		j=0;
		for(j=2;j<=col-3;j+=5)
		{
			for(i=0;i<=row-3;i+=3)
			{
				cont=0;x=0;sw=0;

				if(ptr2[i][j-1]==1||ptr2[i][j]==1||ptr2[i][j+1]==1||ptr2[i][j-2]==1||ptr2[i][j+2]==1)
				{
					cont++; x=i;sw=j;
					while(i<row-3)
					{
						if(ptr2[i+1][sw-1]==1||ptr2[i+1][sw]==1||ptr2[i+1][sw+1]==1||ptr2[i+1][sw-2]==1||ptr2[i+1][sw+2]==1)
						{
							if(ptr2[i+1][sw-1]==1) sw=sw-1;
							else if(ptr2[i+1][sw]==1) sw=sw;
							else if(ptr2[i+1][sw+1]==1) sw=sw+1;
							else if(ptr2[i+1][sw+2]==1) sw=sw+2;
							else if(ptr2[i+1][sw-2]==1) sw=sw-2;
							cont++; i++;
						}
						else if(ptr2[i+2][sw-1]==1||ptr2[i+2][sw]==1||ptr2[i+2][sw+1]==1||ptr2[i+2][sw-2]==1||ptr2[i+2][sw+2]==1)
						{
							if(ptr2[i+2][sw-1]==1) sw=sw-1;
							else if(ptr2[i+2][sw]==1) sw=sw;
							else if(ptr2[i+2][sw+1]==1) sw=sw+1;
							else if(ptr2[i+2][sw+2]==1) sw=sw+2;
							else if(ptr2[i+2][sw-2]==1) sw=sw-2;
							cont+=2; i+=2;
						}
						else	break ;//out while
					}
				}

				if(cont>t3)
				{
					*di=i;
					*dj=j;
					*dx=x;
					*dsw=sw;

					mid_x=(x+i)/2;
					mid_y=(j+sw)/2;

					*midx=mid_x;
					*midy=mid_y;

					break;//inside for
				}
			}

			if(cont>t3)    break; //outside for

		}


	}
  
  
   
	/*---Dynamic Memory Allocation : delete ---*/

	for(int DMA_i=0; DMA_i<row; DMA_i++)    delete ptr2[DMA_i];
	delete ptr2;

} //stairs_find 5 end

