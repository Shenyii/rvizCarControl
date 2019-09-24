#include "navi_ac_terminate.h"

using namespace std;

pthread_mutex_t hMutex = PTHREAD_MUTEX_INITIALIZER;

extern int obstacle;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
ros::Publisher pub;
ros::Publisher cmd;
float goal0[4] = {-0.210,-6.884,0.704,0.710};
float goal1[4] = {-3.168,-7.140,0.000,1.000};
float goal2[4] = {4.822,-6.962,0.715,0.699};
float goal3[4] = {4.667,0.228,0.887,-0.461};
float goal4[4] = {5.854,3.400,-0.718,0.696};

void * Thread_obstacle_handle(void *)
{
    lidar_data_handle point_data_handle;
    ROS_INFO("Hellow world");
}

void Pthread_obstacle_handle()
{
    pthread_t pid_usart;
    printf("topic thread start...\n");
    int pthread_spin = pthread_create(&pid_usart, NULL,Thread_obstacle_handle, NULL);
}

void stop()
{
    geometry_msgs::Twist volacity;
    volacity.linear.x = 0.0;
    volacity.angular.z = 0.0;
    cmd.publish(volacity);
}

void run_mode1(float *target_pose)
{
    tf::TransformListener listener;
    tf::StampedTransform transform;
    geometry_msgs::Twist volacity;
    listener.waitForTransform("/map","/base_footprint",ros::Time(0),ros::Duration(3));
    while(1)
    {
        listener.lookupTransform("/map","/base_footprint",ros::Time(0),transform);
        float x = transform.getOrigin().x();
        float y = transform.getOrigin().y();
        float oz = transform.getRotation().getZ();
        float ow = transform.getRotation().getW();
        float distant2;
        float angle1,angle2,angle3,angle;
        tf::Vector3 v0(1.0,0.0,0.0);
        tf::Vector3 v1(target_pose[0] - x,target_pose[1] - y,0);
        angle1 = tfAngle(v0,v1);
        if(v1[1] < 0) angle1 = 0 - angle1;
        angle2 = 2 * acos(ow);
        if(angle2 > 3.14159265) angle2 = angle2 - 2 * 3.14159265;
        if(oz < 0) angle2 = 0 - angle2;
        angle = angle2 - angle1;
        angle3 = 2 * acos(target_pose[3]);
        if(angle3 > 3.14159265) angle3 = angle3 - 2 * 3.14159265;
        if(target_pose[2] < 0) angle3 = 0 - angle3;
        distant2 = (x - target_pose[0]) * (x - target_pose[0]) + (y - target_pose[1]) * (y - target_pose[1]);
        if(distant2 <= 0.0025)
        {
            if(angle2 - angle3 >= 0.1)
            {
                ROS_INFO("The angle is biger:%f,%f",angle2,angle3);
                volacity.linear.x = 0.0;
                volacity.angular.z = -0.4;
                if(obstacle == 0) cmd.publish(volacity);
                else stop();
            }
            else if(angle2 - angle3 <= -0.1)
            {
                ROS_INFO("The angle is smaller:%f,%f",angle2,angle3);
                volacity.linear.x = 0.0;
                volacity.angular.z = 0.4;
                if(obstacle == 0) cmd.publish(volacity);
                else stop();
            }
            else
            {
                volacity.linear.x = 0.0;
                volacity.angular.z = 0.0;
                if(obstacle == 0) cmd.publish(volacity);
                else stop();
                ROS_INFO("The angle is perfect !");
                break;
            }
        }
        else
        {
            if(angle >= 0.1)
            {
                ROS_INFO("biger %f,%f",angle2,angle1);
                volacity.linear.x = 0.0;
                volacity.angular.z = -0.4;
                if(obstacle == 0) cmd.publish(volacity);
                else stop();
            }
            else if(angle <= -0.1)
            {
                ROS_INFO("smaller %f,%f",angle2,angle1);
                volacity.linear.x = 0.0;
                volacity.angular.z = 0.4;
                if(obstacle == 0) cmd.publish(volacity);
                else stop();
            }
            else
            {
                ROS_INFO("Go Go Go !");
                volacity.linear.x = 0.15;
                volacity.angular.z = 0.0;
                if(obstacle == 0) cmd.publish(volacity);
                else stop();
            }
        }
    }
}

void setHome()
{
    geometry_msgs::PoseWithCovarianceStamped msg_poseinit;
    msg_poseinit.header.frame_id = "map";
    msg_poseinit.header.stamp = ros::Time::now();
    msg_poseinit.pose.pose.position.x = goal0[0];
    msg_poseinit.pose.pose.position.y = goal0[1];
    msg_poseinit.pose.pose.orientation.z = goal0[2];
    msg_poseinit.pose.pose.orientation.w = goal0[3];
    pub.publish(msg_poseinit);
    ros::Duration(1.0).sleep();
    pub.publish(msg_poseinit);
    ros::Duration(1.0).sleep();
    pub.publish(msg_poseinit);
    ros::Duration(1.0).sleep();
}

int setGoal(float *goal_position)
{
    MoveBaseClient ac("move_base",true);
    move_base_msgs::MoveBaseGoal goal;
    tf::TransformListener listener;
    tf::StampedTransform transform;
    listener.waitForTransform("/map","/base_footprint",ros::Time(0),ros::Duration(3));
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server");
    }
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = goal_position[0];
    goal.target_pose.pose.position.y = goal_position[1];
    goal.target_pose.pose.orientation.z = goal_position[2];
    goal.target_pose.pose.orientation.w = goal_position[3];

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
    listener.lookupTransform("/map","/base_footprint",ros::Time(0),transform);
    float x = transform.getOrigin().x();
    float y = transform.getOrigin().y();
    float oz = transform.getRotation().getZ();
    float ow = transform.getRotation().getW();
    while(1)
    {
        listener.lookupTransform("/map","/base_footprint",ros::Time(0),transform);
        x = transform.getOrigin().x();
        y = transform.getOrigin().y();
        float distant2;
        distant2 = (x - goal_position[0]) * (x - goal_position[0]) + (y - goal_position[1]) * (y - goal_position[1]);
        if(distant2 <= 0.01)
        {
            ac.cancelAllGoals();
            break;
        }
    }
    run_mode1(goal_position);

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("You have arrived to the goal reason");
        return 1;
    }
    else
    {
        ROS_INFO("The base failed for some reason");
        return 0;
    }
}

int other_main(float x,float y,float w,float oz)
{
    //ros::init(argc, argv, "navigation_model");
    ros::NodeHandle n;
    pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",1);
    cmd = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);

    Pthread_obstacle_handle();
    usleep(1000*1);
    ROS_INFO("Hellow world!");

    char home_position;
    float robot_goal_position[4];
    robot_goal_position[0] = x;
    robot_goal_position[1] = y;
    robot_goal_position[2] = oz;
    robot_goal_position[3] = w;
    setGoal(robot_goal_position);

    stop_flag = 1;
    while(stop_flag == 1) {}
    // while(1)
    // {
    //     ROS_INFO("If you are in the home,press the 'y' to continue,'n' is set by hand");
    //     cin >> home_position;
    //     if(home_position == 'y')
    //     {
    //         setHome();
    //         break;
    //     }
    //     if(home_position == 'n')
    //     {
    //         ROS_INFO("Please set home position by hand.");
    //         break;
    //     }
    //     else
    //         ROS_INFO("Please put robot on home place.");
    // }
    
    // while(ros::ok())
    // {
    //     ROS_INFO("Where are you going?(0,1,2,3,4)");
    //     int position_num;
    //     cin >> position_num;
    //     switch (position_num)
    //     {
    //         case 0:
    //         {
    //             setGoal(goal0);
    //             break;
    //         }

    //         case 1:
    //         {
    //             setGoal(goal1);
    //             break;
    //         }

    //         case 2:
    //         {
    //             setGoal(goal2);
    //             break;
    //         }

    //         case 3:
    //         {
    //             setGoal(goal3);
    //             break;
    //         }

    //         case 4:
    //         {
    //             setGoal(goal4);
    //             break;
    //         }
                
    //         default:
    //         {
    //             ROS_INFO("The program will stop!");
    //             return 0;
    //         }
    //     }
    // }
    return 0;
}
