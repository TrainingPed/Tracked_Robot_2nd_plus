#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

double x = 0.0;
double y = 0.0;
double th = 0.0;
int clearpath=0;
void clearpath_callback(const std_msgs::Int32::ConstPtr &msg)
{
    clearpath=msg->data;
}
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    tf::Pose pose;
    tf::poseMsgToTF(msg->pose.pose, pose);
    th = tf::getYaw(pose.getRotation());

}
main (int argc, char **argv)
{
    ros::init (argc, argv, "showpath");

    ros::NodeHandle ph;
    ros::Publisher path_pub = ph.advertise<nav_msgs::Path>("odom_trajectory",1, true);
    ros::Subscriber get_pose = ph.subscribe("odom", 1, &odom_callback);
    ros::Subscriber clear_path = ph.subscribe("pathclear",1,&clearpath_callback);

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
//    last_time = ros::Time::now();

    nav_msgs::Path path;
    //nav_msgs::Path path;
    path.header.stamp=current_time;
    path.header.frame_id="map";




    ros::Rate loop_rate(1);
    while (ros::ok())
    {

        if(clearpath)
        {
            path.poses.clear();
        }
        else
        {
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
        }

        path_pub.publish(path);
        ros::spinOnce();               // check for incoming messages

        last_time = current_time;
        loop_rate.sleep();

    }

    return 0;
}
