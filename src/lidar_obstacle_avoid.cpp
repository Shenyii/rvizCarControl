#include "lidar_obstacle_avoid.h"
#include <iostream>

using namespace std;

int obstacle = 0;
int stop_flag = 0;

void lidar_data_handle::point_cb(const sensor_msgs::LaserScan::ConstPtr& in_cloud)
{
    int obstacle_point = 0;
    for(int i = 360;i < 720;i++)
    {
        if(in_cloud->ranges[i] < 0.3)
        {
            if((in_cloud->ranges[i] * cos((i - 360) * 3.1415926 / 720.0 )) < 0.17)
            {
                obstacle_point++;
            }
        }
    }

    if(obstacle_point > 10) obstacle = 1;
    else obstacle = 0;
}

lidar_data_handle::lidar_data_handle()
{
    sub_point_cloud = n.subscribe("/scan",5,&lidar_data_handle::point_cb,this);
    ROS_INFO("Checking the obstacle.");
    while(ros::ok())
    {
        if(stop_flag == 1)
        {
            stop_flag = 0;
            return;
        }
        ros::spinOnce();
    }
}

lidar_data_handle::~lidar_data_handle()
{}

