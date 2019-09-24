#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>
#include <stdio.h>
#include <math.h>
#include "lidar_obstacle_avoid.h"
#include <pthread.h>

extern int obstacle;
extern int stop_flag;

void * Thread_obstacle_handle(void *);
void Pthread_obstacle_handle();
void stop();
void run_mode1(float *target_pose);
void setHome();
int setGoal(float *goal_position);
int other_main(float x,float y,float w,float oz);
