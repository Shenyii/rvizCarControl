#ifndef OBJECT_FOLLOW_H
#define OBJECT_FOLLOW_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
// PCL specific includes 。PCL 的相关的头文件
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
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

#include <QPushButton>
#include <QTextEdit>

class Object_Follow: public QWidget
{
Q_OBJECT
public:
    Object_Follow(QWidget* parent = 0);
    ~Object_Follow();
    int start();
    int quit();
    void run();
    pthread_t getTid();
    bool isAlive();

public Q_SLOTS:
    void clickButtonFollow();

private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    void cloudCb(const sensor_msgs::LaserScanConstPtr& cloud_msg);

    static void * start_func(void * arg);
    pthread_t m_tid;
    bool m_isAlive;

    QPushButton* button_follow_;
    QTextEdit* text_follow_;

    int stop_flag_;
};

#endif