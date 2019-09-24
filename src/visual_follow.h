#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
// PCL specific includes 。PCL 的相关的头文件
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
//滤波的头文件
#include <pcl/filters/voxel_grid.h>
//申明发布器
#include <iostream>
#include <vector>
#include <algorithm>
#include <pthread.h>
#include <iostream>
#include <signal.h>

class visualFollow
{
public:
    visualFollow();
    ~visualFollow();
    int start();
    int quit();
    void run();
    pthread_t getTid();
    bool isAlive();

private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    void cloudCb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

    static void * start_func(void * arg);
    pthread_t m_tid;
    bool m_isAlive;
};