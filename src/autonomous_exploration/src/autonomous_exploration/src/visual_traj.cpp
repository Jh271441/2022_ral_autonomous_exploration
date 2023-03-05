//
// Created by cjh on 23-3-5.
//
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PointStamped.h"

void odomCallBack(const nav_msgs::Odometry::ConstPtr & msg){

}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "traj_visualizer");
    ros::NodeHandle n;

    ros::Subscriber odom_sub_ = n.subscribe("/odom", 10, odomCallBack);
    ros::spin();
    return 0;
}