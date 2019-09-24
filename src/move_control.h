#ifndef MOVE_CONTROL_H
#define MOVE_CONTROL_H

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <pthread.h>
#include <signal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <unistd.h>
#include "tf_listerner.h"
#include "pose.h"

using namespace std;

class Move_Control
{
public:
    Move_Control();
    ~Move_Control();
    void getPath(vector<Pose> path_point);
    int start();
    int quit();
    void run();
    pthread_t getTid();
    bool isAlive();
    float get_bias();
    void send_velocity();
    void stop_car();

private:
    Tf_Listerner* tf_listen_;

    vector<geometry_msgs::Pose> path_point_;
    geometry_msgs::Pose current_position_;
    float bias_of_car_and_path_;
    Pose goal_pose_;
    float angle_bias_;
    float distance_bias_;

    ros::NodeHandle n_;
    ros::Publisher pub_velocity_;
    int velocity_send_state_;
    void goToFirstPose();
    void goToFinalyPose();

    // pthread_t run_mode_tid_;
    // void pthreadRunModeOfFirstAndEnd();
    // static void* threadRunModeOfFirstAndEnd(void *arg);
    void RunModeOfFirstAndEnd();
    int runModelOfDestination(float *target_pose);

    static void *start_func(void *arg);
    pthread_t m_tid_;
    bool m_isAlive_;
};
#endif