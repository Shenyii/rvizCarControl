#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

using namespace std;

extern int obstacle;

class lidar_data_handle
{
public:
    lidar_data_handle();
    ~lidar_data_handle();

private:
    ros::NodeHandle n;
    ros::Subscriber sub_point_cloud;
    void point_cb(const sensor_msgs::LaserScan::ConstPtr& in_cloud);
};