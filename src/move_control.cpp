#include "move_control.h"

Move_Control::Move_Control()
: velocity_send_state_(0),distance_bias_(0),angle_bias_(0)
{
    pub_velocity_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel",1);

    tf_listen_ = new Tf_Listerner("map","base_footprint");

    start();
}

Move_Control::~Move_Control()
{
    if(m_isAlive_)
    {
        pthread_kill(m_tid_,0);
    }
}

void Move_Control::getPath(vector<Pose> path_point)
{
    velocity_send_state_ = 1;
    path_point_.clear();
    for(int i = 0;i < path_point.size();i++)
    {
        geometry_msgs::Pose pose;
        pose.position.x = path_point[i].x;
        pose.position.y = path_point[i].y;
        pose.position.z = 0;
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = path_point[i].oz;
        pose.orientation.w = path_point[i].ow;
        if((i >= 0) && (i < path_point.size() - 1))
        {
            float x;
            float y;
            float ow;
            float oz;
            x = path_point[i + 1].x - pose.position.x;
            y = path_point[i + 1].y - pose.position.y;
            ow = cos(acos(x / sqrt(x * x + y * y)) / 2);
            oz = sqrt(1 - ow * ow);
            if(y < 0) oz = 0 - oz;
            pose.orientation.z = oz;
            pose.orientation.w = ow;
        }

        path_point_.push_back(pose);
    }
}

int Move_Control::start()
{
    ROS_INFO("Start a new thread to trace path.");
    if(pthread_create(&m_tid_,NULL,start_func,(void*)this) != 0)
    {
        ROS_INFO("Start a new thread failed!");
        return -1;
    }
}

int Move_Control::quit()
{
    ROS_INFO("Quit the thread tid.");
    m_isAlive_ = false;
    return 0;
}

void Move_Control::run()
{
    send_velocity();
}

pthread_t Move_Control::getTid()
{
    return m_tid_;
}

bool Move_Control::isAlive()
{
    return m_isAlive_;
}

float Move_Control::get_bias()
{
    ros::Duration(0.1).sleep();;
    float bias;
    int n = 0;
    float min_distance = 1.0;
    float x;
    float y;
    float oz;
    float ow;
    x = current_position_.position.x;
    y = current_position_.position.y;
    oz = current_position_.orientation.z;
    ow = current_position_.orientation.w;
    if(ow < 0)
    {
        ow = 0 - ow;
        oz = 0 - oz;
    }

    for(int i = 0;i < path_point_.size();i++)
    {
        float distance;
        distance = sqrt((x-path_point_[i].position.x)*(x-path_point_[i].position.x)+(y-path_point_[i].position.y)*(y-path_point_[i].position.y));

        if(distance < min_distance)
        {
            min_distance = distance;
            n = i;
        }
    }

    float min_point_x;
    float min_point_y;
    float min_point_oz;
    float min_point_ow;
    min_point_x = path_point_[n].position.x;
    min_point_y = path_point_[n].position.y;
    min_point_oz = path_point_[n].orientation.z;
    min_point_ow = path_point_[n].orientation.w;

    tf::Quaternion point_in_map(0,0,min_point_oz,min_point_ow);
    tf::Matrix3x3 matrix;
    matrix.setRotation(point_in_map);
    if(matrix[0][1] * (x - min_point_x) + matrix[1][1] * (y - min_point_y) < 0)
    {
        min_distance = 0 - min_distance;
    }

    float angle_sin;
    float angle_bias;
    float p_c;
    float p_s;
    if(oz > 0)
    {
        p_c = 2.0 * ow * ow - 1.0;
        p_s = 2.0 * sqrt(1.0 - ow * ow) * ow;
    }
    else
    {
        p_c = 2.0 * ow * ow - 1.0;
        p_s = 0.0 - 2.0 * sqrt(1 - ow * ow) * ow;
    }
    if(min_point_oz > 0)
    {
        angle_sin = (2.0*min_point_ow*min_point_ow-1.0)*p_s-2.0*sqrt(1.0-min_point_ow*min_point_ow)*min_point_ow*p_c;
    }
    else
    {
        angle_sin = (2.0*min_point_ow*min_point_ow-1.0)*p_s+2.0*sqrt(1.0-min_point_ow*min_point_ow)*min_point_ow*p_c;
    }
    angle_bias = asin(angle_sin);
    if(min_point_oz > 0)
    {
        if((2*min_point_ow*min_point_ow-1)*p_c+2*sqrt(1-min_point_ow*min_point_ow)*min_point_ow*p_s < 0)
        {
            if(angle_bias > 0) angle_bias = 3.14159265 - angle_bias;
            else angle_bias = 0 - 3.14159265 - angle_bias;
        }
    }
    else
    {
        if((2*min_point_ow*min_point_ow-1)*p_c-2*sqrt(1-min_point_ow*min_point_ow)*min_point_ow*p_s < 0)
        {
            if(angle_bias > 0) angle_bias = 3.14159265 - angle_bias;
            else angle_bias = 0 - 3.14159265 - angle_bias;
        }
    }

    bias = min_distance * 4.0 + angle_bias;
    angle_bias_ = angle_bias;
    distance_bias_ = min_distance;

    // ROS_INFO("(%f,%f,%f)",matrix[0][0],matrix[0][1],matrix[0][2]);
    // ROS_INFO("(%f,%f,%f)",matrix[1][0],matrix[1][1],matrix[1][2]);
    // ROS_INFO("(%f,%f,%f)",matrix[2][0],matrix[2][1],matrix[2][2]);
    // ROS_INFO("current(%f,%f,%f,%f)",x,y,ow,oz);
    // ROS_INFO("       (%f,%f,%f,%f)",min_point_x,min_point_y,min_point_ow,min_point_oz);
    // ROS_INFO("thebias(%f,%f,%f).",distance_bias_,angle_bias_,bias);

    return bias;
}

void Move_Control::send_velocity()
{
    geometry_msgs::Twist velocity;
    //ROS_INFO("test test test!!!");
    while(ros::ok())
    {
        while(!velocity_send_state_)
        {
            ros::Duration(1).sleep();
        }
        
        //first go to the start pose;
        goToFirstPose();
        if(velocity_send_state_ == 0)
        {
            continue;
        }

        while(ros::ok())
        {
            //terminal condition
            float det_x = tf_listen_->x() - path_point_[path_point_.size() - 1].position.x;
            float det_y = tf_listen_->y() - path_point_[path_point_.size() - 1].position.y;
            if((det_x * det_x + det_y * det_y) < 0.09)
            {
                break;
            }

            current_position_.position.x = tf_listen_->x();
            current_position_.position.y = tf_listen_->y();
            current_position_.position.z = 0;
            current_position_.orientation.x = 0;
            current_position_.orientation.y = 0;
            current_position_.orientation.z = tf_listen_->oz();
            current_position_.orientation.w = tf_listen_->ow();

            float bias;
            bias = get_bias();
            if(angle_bias_ > 0.5)
            {
                velocity.linear.x = 0;
                velocity.linear.y = 0;
                velocity.linear.z = 0;
                velocity.angular.x = 0;
                velocity.angular.y = 0;
                velocity.angular.z = -0.3;
                if(velocity.angular.z > 0.3) velocity.angular.z = 0.3;
                if(velocity.angular.z < -0.3) velocity.angular.z = -0.3;
                pub_velocity_.publish(velocity);
                cout << "the angle is too big!" << endl;
                continue;
            }
            else if(angle_bias_ < -0.5)
            {
                velocity.linear.x = 0;
                velocity.linear.y = 0;
                velocity.linear.z = 0;
                velocity.angular.x = 0;
                velocity.angular.y = 0;
                velocity.angular.z = 0.3;
                if(velocity.angular.z > 0.3) velocity.angular.z = 0.3;
                if(velocity.angular.z < -0.3) velocity.angular.z = -0.3;
                pub_velocity_.publish(velocity);
                cout << "the angle is too small!" << endl;
                continue;
            }

            if(bias > 1.5) bias = 1.5;
            if(bias < -1.5) bias = -1.5;
            velocity.linear.x = 0.15;
            velocity.linear.y = 0;
            velocity.linear.z = 0;
            velocity.angular.x = 0;
            velocity.angular.y = 0;
            velocity.angular.z = 0 - bias;
            if(velocity.angular.z > 0.3) velocity.angular.z = 0.3;
            if(velocity.angular.z < -0.3) velocity.angular.z = -0.3;
            pub_velocity_.publish(velocity);
        }

        stop_car();

        //finaly go to end pose
        goToFinalyPose();

        velocity_send_state_ = 0;
    }
}

void Move_Control::stop_car()
{
    geometry_msgs::Twist velocity;
    velocity_send_state_ = 0;
    velocity.linear.x = 0;
    velocity.linear.y = 0;
    velocity.linear.z = 0;
    velocity.angular.x = 0;
    velocity.angular.y = 0;
    velocity.angular.z = 0;
    for(int i = 0; i < 5;i++)
    {
        pub_velocity_.publish(velocity);
    }
    ROS_INFO("Stop the AGV.");
}

void * Move_Control::start_func(void *arg)
{
    Move_Control *ptr = (Move_Control*) arg;
    ptr->run();
    return NULL;
}

void Move_Control::goToFirstPose()
{
    goal_pose_.x = path_point_[0].position.x;
    goal_pose_.y = path_point_[0].position.y;
    goal_pose_.oz = path_point_[0].orientation.z;
    goal_pose_.ow = path_point_[0].orientation.w;
    RunModeOfFirstAndEnd();
}

void Move_Control::goToFinalyPose()
{
    goal_pose_.x = path_point_[path_point_.size() - 1].position.x;
    goal_pose_.y = path_point_[path_point_.size() - 1].position.y;
    goal_pose_.oz = path_point_[path_point_.size() - 1].orientation.z;
    goal_pose_.ow = path_point_[path_point_.size() - 1].orientation.w;
    RunModeOfFirstAndEnd();
}

// void Move_Control::pthreadRunModeOfFirstAndEnd()
// {
//     ROS_INFO("Start a new thread of go to goal pose.");
//     if(pthread_create(&run_mode_tid_,NULL,threadRunModeOfFirstAndEnd,(void*)this) != 0)
//     {
//         ROS_INFO("Start a new thread failed!");
//         return;
//     }
// }

// void* Move_Control::threadRunModeOfFirstAndEnd(void *arg)
// {
//     Move_Control *ptr = (Move_Control*) arg;
//     ptr->RunModeOfFirstAndEnd();
//     return NULL;
// }

void Move_Control::RunModeOfFirstAndEnd()
{
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
    move_base_msgs::MoveBaseGoal goal;
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server");
    }
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = goal_pose_.x;
    goal.target_pose.pose.position.y = goal_pose_.y;
    goal.target_pose.pose.orientation.z = goal_pose_.oz;
    goal.target_pose.pose.orientation.w = goal_pose_.ow;

    ROS_INFO("x,y,ow,oz=(%f,%f,%f,%f).",goal_pose_.x,goal_pose_.y,goal_pose_.ow,goal_pose_.oz);
    ros::Duration(3).sleep();
    ac.sendGoal(goal);
    float x = tf_listen_->x();
    float y = tf_listen_->y();
    float oz = tf_listen_->oz();
    float ow = tf_listen_->ow();
    float distant2;
    while(ros::ok())
    {
        x = tf_listen_->x();
        y = tf_listen_->y();
        distant2 = (x - goal_pose_.x) * (x - goal_pose_.x) + (y - goal_pose_.y) * (y - goal_pose_.y);
        if(distant2 <= 0.01)
        {
            ac.cancelAllGoals();
            break;
        }

        if(ac.getState().toString() == "PENDING")
        {
            //ROS_INFO("action mode 0");
        }
        else if(ac.getState().toString() == "ACTIVE")
        {
            //ROS_INFO("action mode 1");
        }
        else if(ac.getState().toString() == "RECALLED")
        {
            //ROS_INFO("action mode 2");
            stop_car();
            return;
        }
        else if(ac.getState().toString() == "REJECTED")
        {
            //ROS_INFO("action mode 3");
            stop_car();
            return;
        }
        else if(ac.getState().toString() == "PREEMPTED")
        {
            //ROS_INFO("action mode 4");
            stop_car();
            return;
        }
        else if(ac.getState().toString() == "ABORTED")
        {
            //ROS_INFO("action mode 5");
            stop_car();
            return;
        }
        else if(ac.getState().toString() == "SUCCEEDED")
        {
            //ROS_INFO("action mode 6");
            stop_car();
            return;
        }
        else if(ac.getState().toString() == "LOST")
        {
            //ROS_INFO("action mode 7");
            stop_car();
            return;
        }
    }
    ros::Duration(1).sleep();

    float goal_position[4];
    goal_position[0] = goal_pose_.x;
    goal_position[1] = goal_pose_.y;
    goal_position[2] = goal_pose_.oz;
    goal_position[3] = goal_pose_.ow;
    ROS_INFO("Begin other running model.%f.",distant2);
    if(runModelOfDestination(goal_position))
    {
        //return 1;
    }
}

int Move_Control::runModelOfDestination(float *target_pose)
{
    geometry_msgs::Twist volacity;
    while(ros::ok())
    {
        float x = tf_listen_->x();
        float y = tf_listen_->y();
        float oz = tf_listen_->oz();
        float ow = tf_listen_->ow();
        //ROS_INFO("xy,oz,ow=(%f,%f,%f,%f)",target_pose[0],target_pose[1],target_pose[2],target_pose[3]);
        //ros::Duration(2).sleep();

        float distant2;
        float angle1,angle2,angle3,angle;
        tf::Vector3 v0(1.0,0.0,0.0);
        tf::Vector3 v1(target_pose[0] - x,target_pose[1] - y,0);
        angle1 = tfAngle(v0,v1);
        if(v1[1] < 0) angle1 = 0 - angle1;
        angle2 = 2 * acos(ow);
        if(angle2 > 3.14159265) angle2 = angle2 - 2 * 3.14159265;
        if(angle2 < -3.14159265) angle2 = angle2 + 2 * 3.14159265;
        if(oz < 0) angle2 = 0 - angle2;
        angle = angle2 - angle1;
        angle3 = 2 * acos(target_pose[3]);
        if(angle3 > 3.14159265) angle3 = angle3 - 2 * 3.14159265;
        if(angle3 < -3.14159265) angle3 = angle3 + 2 * 3.14159265;
        if(target_pose[2] < 0) angle3 = 0 - angle3;
        distant2 = (x - target_pose[0]) * (x - target_pose[0]) + (y - target_pose[1]) * (y - target_pose[1]);
        if(distant2 <= 0.0025)
        {
            if(angle2 - angle3 >= 0.1)
            {
                //ROS_INFO("The angle is biger:%f,%f",angle2,angle3);
                volacity.linear.x = 0.0;
                volacity.angular.z = -0.4;
                pub_velocity_.publish(volacity);
            }
            else if(angle2 - angle3 <= -0.1)
            {
                //ROS_INFO("The angle is smaller:%f,%f",angle2,angle3);
                volacity.linear.x = 0.0;
                volacity.angular.z = 0.4;
                pub_velocity_.publish(volacity);
            }
            else
            {
                volacity.linear.x = 0.0;
                volacity.angular.z = 0.0;
                pub_velocity_.publish(volacity);
                ROS_INFO("The angle is perfect !");
                break;
            }
        }
        else
        {
            if(angle >= 0.1)
            {
                //ROS_INFO("biger %f,%f",angle2,angle1);
                volacity.linear.x = 0.0;
                volacity.angular.z = -0.4;
                pub_velocity_.publish(volacity);
            }
            else if(angle <= -0.1)
            {
                //ROS_INFO("smaller %f,%f",angle2,angle1);
                volacity.linear.x = 0.0;
                volacity.angular.z = 0.4;
                pub_velocity_.publish(volacity);
            }
            else
            {
                //ROS_INFO("Go Go Go !");
                volacity.linear.x = 0.15;
                volacity.angular.z = 0.0;
                pub_velocity_.publish(volacity);
            }
        }
    }
    return 1;
}