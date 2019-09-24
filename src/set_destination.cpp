#include "set_destination.h"

std::vector<Destination_Pose> destination_list_g;
Destination_Pose destination_g;
Tf_Listerner position_map_basefootprint_g("map","base_footprint");

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_t;
interactive_markers::MenuHandler menu_handler;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
ros::Publisher cmd_g;
int whether_arrived_destination_g = 0;//0-no task 1-task is running
ros::Publisher ros_pub_pose_goal_g;
ros::Subscriber task_state_g;// = nh.subscribe("topic_name", 1000, subCallback);


Set_Destination::Set_Destination(QWidget* parent)
  : QWidget(parent),type_of_operator_(0),loop_task_ptr_(0)
{
    state_and_prompt_ = new QTextEdit(this);
    state_and_prompt_->setGeometry(QRect(0,0,300,30));
    state_and_prompt_->setText("Display state and prompt.");

    button_loop_task_ = new QPushButton(this);
    button_loop_task_->setGeometry(QRect(0,35,130,30));
    button_loop_task_->setText("loop task");
    button_loop_task_->setStyleSheet("background-color:rgb(102,0,102);" "color:rgb(255,255,255);");
    text_loop_task_ = new QTextEdit(this);
    text_loop_task_->setGeometry(QRect(135,35,165,30));
    text_loop_task_->setText("Please input loop task(eg:0-1)");

    button_create_destination_ = new QPushButton(this);
    button_create_destination_->setGeometry(QRect(0,70,300,30));
    button_create_destination_->setText("create destination");
    button_create_destination_->setStyleSheet("background-color:rgb(102,0,102);" "color:rgb(255,255,255);");

    // button_send_destination_ = new QPushButton(this);
    // button_send_destination_->setGeometry(QRect(170,70,130,30));
    // button_send_destination_->setText("send destination");

    // button_adjust_destination_ = new QPushButton(this);
    // button_adjust_destination_->setGeometry(QRect(0,105,130,30));
    // button_adjust_destination_->setText("adjust destination");

    connect(button_loop_task_,SIGNAL(clicked(bool)),this,SLOT(clickButtonLoopTask()));
    connect(text_loop_task_,SIGNAL(textChanged()),this,SLOT(updateLoopTaskText()));
    connect(button_create_destination_,SIGNAL(clicked(bool)),this,SLOT(clickButtonCreateDestination()));
    //connect(button_send_destination_,SIGNAL(clicked(bool)),this,SLOT(clickButtonSendDestination()));
    //connect(button_adjust_destination_,SIGNAL(clicked(bool)),this,SLOT(clickButtonAdjustDestination()));
    QTimer* run_timer = new QTimer(this);
    connect(run_timer,SIGNAL(timeout()),this,SLOT(typeOfOperator()));
    run_timer->start(10);

    static int pthreadRosServerNum = 0;
    pthreadRosServerNum++;
    if(pthreadRosServerNum == 1) 
    {
        ros_pub_pose_goal_g = n_.advertise<geometry_msgs::Pose>("/destination_pose",1);
        task_state_g = n_.subscribe("/destination_task_state",2,taskStateCallback);
        PthreadRosServer();
    }
}

void Set_Destination::clickButtonLoopTask()
{
    ROS_INFO("Begin the loop task.");
    if(type_of_operator_ != 1) type_of_operator_ = 1;
    else if(type_of_operator_ == 1) 
    {
        type_of_operator_ = 0;
        whether_arrived_destination_g = 0;
    }
}

void Set_Destination::updateLoopTaskText()
{
    ROS_INFO("Update lool task.");
    ROS_INFO("Loop task is %s.",text_loop_task_->toPlainText().toStdString().c_str());
    type_of_operator_ = 2;
}

void Set_Destination::clickButtonCreateDestination()
{
    ROS_INFO("Create destination.");
    type_of_operator_ = 3;
    Destination_Pose pose_init(0,0,1,0);
    destination_list_g.push_back(pose_init);
    displayDestination(destination_list_g);
    //server_t->applyChanges();
    type_of_operator_ = 0;
}

// void Set_Destination::clickButtonSendDestination()
// {
//     ROS_INFO("Send destination.");
//     type_of_operator_ = 4;
// }

// void Set_Destination::clickButtonAdjustDestination()
// {
//     ROS_INFO("Adjust destination.");
//     type_of_operator_ = 5;
// }

void Set_Destination::typeOfOperator()
{
    if(type_of_operator_ == 0)
    {
        state_and_prompt_->setText("Can't found valid operator.");
    }
    else if(type_of_operator_ == 1)
    {
        if(whether_arrived_destination_g == 0)
        {
            state_and_prompt_->setText("Begin the loop task.0");
        }
        else if(whether_arrived_destination_g == 1)
        {
            state_and_prompt_->setText("Begin the loop task.1");
        }
        
        beginLoopTask();
    }
    else if(type_of_operator_ == 2)
    {}
    else if(type_of_operator_ == 3)
    {
        state_and_prompt_->setText("Has create the destination.");
    }
}

void Set_Destination::beginLoopTask()
{
    std::string loop_task_s;
    loop_task_s = text_loop_task_->toPlainText().toStdString();
    std::string task_s;
    loop_task_list_.clear();
    for(int i = 0;i < loop_task_s.size();i++)
    {
        if(loop_task_s[i] != '-')
        {
            task_s+=loop_task_s[i];
        }
        else
        {
            loop_task_list_.push_back(atoi(task_s.c_str()));
            task_s = "";
        }
        if(i == loop_task_s.size() - 1) 
        {
            loop_task_list_.push_back(atoi(task_s.c_str()));
        }
    }

    if(loop_task_ptr_ >= loop_task_list_.size()) loop_task_ptr_ = 0;
    if(whether_arrived_destination_g == 0) 
    {
        if(destination_list_g.size() == 0)
        {
            return;
        }

        if(loop_task_list_[loop_task_ptr_] + 1 > destination_list_g.size())
        {
            text_loop_task_->setText("The task is false!");
            return;
        }

        destination_g.x = destination_list_g[loop_task_list_[loop_task_ptr_]].x;
        destination_g.y = destination_list_g[loop_task_list_[loop_task_ptr_]].y;
        destination_g.oz = destination_list_g[loop_task_list_[loop_task_ptr_]].oz;
        destination_g.ow = destination_list_g[loop_task_list_[loop_task_ptr_]].ow;
        ros::Duration(2).sleep();
        geometry_msgs::Pose ros_pose_msg;
        ros_pose_msg.position.x = destination_g.x;
        ros_pose_msg.position.y = destination_g.y;
        ros_pose_msg.orientation.z = destination_g.oz;
        ros_pose_msg.orientation.w = destination_g.ow;
        ros_pub_pose_goal_g.publish(ros_pose_msg);
        whether_arrived_destination_g = 1;
        //pthreadSendDestination();
        loop_task_ptr_++;
    }
}

void Set_Destination::PthreadRosServer()
{
    if(pthread_create(&m_tid,NULL,ThreadRosServer,(void*)this) != 0)
    {
        ROS_INFO("Start a new thread failed!");
        return; 
    }
}

void * Set_Destination::ThreadRosServer(void * arg)
{
    Set_Destination *ptr =(Set_Destination*) arg;
    ptr->runOtherPthread();
    return NULL;
}

void Set_Destination::runOtherPthread()
{
    //ROS_INFO("other pthread begin!!!");
    server_t.reset(new interactive_markers::InteractiveMarkerServer("basic_controls","",false));
    menu_handler.insert("Send the destination",&processFeedback);
    menu_handler.insert("Remove the destination",&processFeedback);
    //Destination_Pose pose_init(0,0,1,0);
    //make6DofMarker(false,visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D,true,pose_init);
    //server_t->applyChanges();
    while(ros::ok())
    {
        ros::spinOnce();
        ros::Duration(1).sleep();
        // ROS_INFO("run run test!!");
    }
    server_t.reset();
}

void displayDestination(std::vector<Destination_Pose> destination_list)
{
    server_t.reset();
    server_t.reset(new interactive_markers::InteractiveMarkerServer("basic_controls","",false));
    for(int i = 0;i < destination_list.size();i++)
    {
        make6DofMarker(destination_list[i],i);
    }
}

//void Set_Destination::make6DofMarker(bool fixed,unsigned int interaction_mode,const tf::Vector3& position,bool show_6dof )
void make6DofMarker(Destination_Pose destination_pose,int number_name)
{
    tf::Vector3 position;
    position = tf::Vector3(destination_pose.x,destination_pose.y,0.05);
    bool fixed = false;
    unsigned int interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
    bool show_6dof = true;

    InteractiveMarker int_marker;
    int_marker.header.frame_id = "map";
    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.pose.orientation.x = 0;
    int_marker.pose.orientation.y = 0;
    int_marker.pose.orientation.z = destination_pose.oz;
    int_marker.pose.orientation.w = destination_pose.ow;
    int_marker.scale = 1;

    //int_marker.name = number_name;
    int_marker.description = "number_name:";
    char name_s[25];
    sprintf(name_s,"%d",number_name);
    int_marker.name = name_s;
    int_marker.description += name_s;

    // insert a box
    makeBoxControl(int_marker);
    int_marker.controls[0].interaction_mode = interaction_mode;

    InteractiveMarkerControl control;

    if ( fixed )
    {
        //int_marker.name += "_fixed";
        //int_marker.description += "\n(fixed orientation)";
        control.orientation_mode = InteractiveMarkerControl::FIXED;
    }

    if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
    {
        std::string mode_text;
        if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D )         mode_text = "MOVE_3D";
        if( interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D )       mode_text = "ROTATE_3D";
        if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D )  mode_text = "MOVE_ROTATE_3D";
        //int_marker.name += "_" + mode_text;
        //int_marker.description = std::string("3D Control") + (show_6dof ? " + 6-DOF controls" : "") + "\n" + mode_text;
    }

    if(show_6dof)
    {
        tf::Quaternion orien(1.0, 0.0, 0.0, 1.0);
        orien.normalize();
        tf::quaternionTFToMsg(orien, control.orientation);
        // control.name = "rotate_x";
        // control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        // int_marker.controls.push_back(control);
        control.name = "move_x";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        orien = tf::Quaternion(0.0, 1.0, 0.0, 1.0);
        orien.normalize();
        tf::quaternionTFToMsg(orien, control.orientation);
        control.name = "rotate_z";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        // control.name = "move_z";
        // control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        // int_marker.controls.push_back(control);

        orien = tf::Quaternion(0.0, 0.0, 1.0, 1.0);
        orien.normalize();
        tf::quaternionTFToMsg(orien, control.orientation);
        // control.name = "rotate_y";
        // control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        // int_marker.controls.push_back(control);
        control.name = "move_y";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);
    }

    server_t->insert(int_marker);
    server_t->setCallback(int_marker.name, &processFeedback);
    if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
        menu_handler.apply( *server_t, int_marker.name );
    server_t->applyChanges();
}

void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    ROS_INFO("The destination position:(%f,%f,%f,%f).",feedback->pose.position.x,feedback->pose.position.y,feedback->pose.orientation.w,feedback->pose.orientation.z);
    int destination_id;
    destination_id = atoi(feedback->marker_name.c_str());
    Destination_Pose change_pose;
    change_pose.x = feedback->pose.position.x;
    change_pose.y = feedback->pose.position.y;
    change_pose.ow = feedback->pose.orientation.w;
    change_pose.oz = feedback->pose.orientation.z;

    switch(feedback->event_type)
    {
        case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
            //ROS_INFO("button click.");
            break;

        case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
            //ROS_INFO("%d",feedback->menu_entry_id);
            if(feedback->menu_entry_id == 1)
            {
                //send destination
                destination_g.x = change_pose.x;
                destination_g.y = change_pose.y;
                destination_g.oz = change_pose.oz;
                destination_g.ow = change_pose.ow;
                geometry_msgs::Pose ros_pose_msg;
                ros_pose_msg.position.x = change_pose.x;
                ros_pose_msg.position.y = change_pose.y;
                ros_pose_msg.orientation.z = change_pose.oz;
                ros_pose_msg.orientation.w = change_pose.ow;
                if(whether_arrived_destination_g == 0) 
                {
                    //pthreadSendDestination();
                    ros_pub_pose_goal_g.publish(ros_pose_msg);
                    whether_arrived_destination_g = 1;
                }
            }
            else if(feedback->menu_entry_id == 2)
            {
                destination_list_g.erase(destination_list_g.begin() + destination_id,destination_list_g.begin() + destination_id + 1);
                displayDestination(destination_list_g);
            }
            break;

        case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
            //ROS_INFO("update the %d pose",destination_id);
            destination_list_g.erase(destination_list_g.begin() + destination_id,destination_list_g.begin() + destination_id + 1);
            destination_list_g.insert(destination_list_g.begin() + destination_id,change_pose);
            break;

        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
            break;

        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
            displayDestination(destination_list_g);
            break;
    }
    server_t->applyChanges();
}

Marker makeCylinder( InteractiveMarker &msg )
{
    Marker marker;

    marker.type = Marker::CYLINDER;
    marker.scale.x = msg.scale * 0.45;
    marker.scale.y = msg.scale * 0.45;
    marker.scale.z = msg.scale * 0.05;
    marker.color.r = 0.1;
    marker.color.g = 0.9;
    marker.color.b = 0.1;
    marker.color.a = 1.0;

    return marker;
}

Marker makeArrow( InteractiveMarker &msg )
{
    Marker marker;

    marker.type = Marker::MESH_RESOURCE;
    marker.mesh_resource = "package://rvizCarControl/media/tringle2.dae";
    marker.pose.position.x = 0.2;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    // marker.scale.x = msg.scale * 0.2;
    // marker.scale.y = msg.scale * 0.05;
    // marker.scale.z = msg.scale * 0.05;
    marker.color.r = 0.9;
    marker.color.g = 0.2;
    marker.color.b = 0.2;
    marker.color.a = 1.0;

    return marker;
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
{
    InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back( makeCylinder(msg) );
    control.markers.push_back( makeArrow(msg) );
    msg.controls.push_back( control );

    return msg.controls.back();
}

void pthreadSendDestination()
{
    //position_map_basefootprint_g = new Tf_Listerner("map","base_footprint");
    pthread_t pid_usart;
    ROS_INFO("Start navigation pthread!");
    int pthread_spin = pthread_create(&pid_usart, NULL,sendDestination,NULL);
}

void *sendDestination(void *)
{
    whether_arrived_destination_g = 1;
    MoveBaseClient ac("move_base",true);
    move_base_msgs::MoveBaseGoal goal;
    ros::NodeHandle n;
    cmd_g = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server");
    }
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = destination_g.x;
    goal.target_pose.pose.position.y = destination_g.y;
    goal.target_pose.pose.orientation.z = destination_g.oz;
    goal.target_pose.pose.orientation.w = destination_g.ow;

    ROS_INFO("x,y,ow,oz=(%f,%f,%f,%f).",destination_g.x,destination_g.y,destination_g.ow,destination_g.oz);
    ros::Duration(3).sleep();
    ac.sendGoal(goal);
    float x = position_map_basefootprint_g.x();
    float y = position_map_basefootprint_g.y();
    float oz = position_map_basefootprint_g.oz();
    float ow = position_map_basefootprint_g.ow();
    float distant2;
    while(ros::ok())
    {
        x = position_map_basefootprint_g.x();
        y = position_map_basefootprint_g.y();
        distant2 = (x - destination_g.x) * (x - destination_g.x) + (y - destination_g.y) * (y - destination_g.y);
        if(distant2 <= 0.01)
        {
            ac.cancelAllGoals();
            break;
        }
        
        if(ac.getState().toString() == "PENDING")
        {
            //ROS_INFO("action mode 0");
            //return NULL;
        }
        else if(ac.getState().toString() == "ACTIVE")
        {
            //ROS_INFO("action mode 1");
        }
        else if(ac.getState().toString() == "RECALLED")
        {
            ROS_INFO("action mode 2");
            whether_arrived_destination_g = 0;
            return NULL;
        }
        else if(ac.getState().toString() == "REJECTED")
        {
            ROS_INFO("action mode 3");
            whether_arrived_destination_g = 0;
            return NULL;
        }
        else if(ac.getState().toString() == "PREEMPTED")
        {
            ROS_INFO("action mode 4");
            whether_arrived_destination_g = 0;
            return NULL;
        }
        else if(ac.getState().toString() == "ABORTED")
        {
            ROS_INFO("action mode 5");
            whether_arrived_destination_g = 0;
            return NULL;
        }
        else if(ac.getState().toString() == "SUCCEEDED")
        {
            ROS_INFO("action mode 6");
            whether_arrived_destination_g = 0;
            return NULL;
        }
        else if(ac.getState().toString() == "LOST")
        {
            ROS_INFO("action mode 7");
            whether_arrived_destination_g = 0;
            return NULL;
        }
    }
    ros::Duration(1).sleep();

    float goal_position[4];
    goal_position[0] = destination_g.x;
    goal_position[1] = destination_g.y;
    goal_position[2] = destination_g.oz;
    goal_position[3] = destination_g.ow;
    ROS_INFO("Begin other running model.%f.",distant2);
    if(runModelOfDestination(goal_position))
    {
        //return 1;
    }
}

int runModelOfDestination(float *target_pose)
{
    geometry_msgs::Twist volacity;
    while(ros::ok())
    {
        float x = position_map_basefootprint_g.x();
        float y = position_map_basefootprint_g.y();
        float oz = position_map_basefootprint_g.oz();
        float ow = position_map_basefootprint_g.ow();
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
                cmd_g.publish(volacity);
            }
            else if(angle2 - angle3 <= -0.1)
            {
                //ROS_INFO("The angle is smaller:%f,%f",angle2,angle3);
                volacity.linear.x = 0.0;
                volacity.angular.z = 0.4;
                cmd_g.publish(volacity);
            }
            else
            {
                volacity.linear.x = 0.0;
                volacity.angular.z = 0.0;
                cmd_g.publish(volacity);
                ROS_INFO("The angle is perfect !");
                whether_arrived_destination_g = 0;
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
                cmd_g.publish(volacity);
            }
            else if(angle <= -0.1)
            {
                //ROS_INFO("smaller %f,%f",angle2,angle1);
                volacity.linear.x = 0.0;
                volacity.angular.z = 0.4;
                cmd_g.publish(volacity);
            }
            else
            {
                //ROS_INFO("Go Go Go !");
                volacity.linear.x = 0.15;
                volacity.angular.z = 0.0;
                cmd_g.publish(volacity);
            }
        }
    }

    whether_arrived_destination_g = 0;
    //position_map_basefootprint_g->stop_tf_listerner();
    return 1;
}

void taskStateCallback(const std_msgs::String::ConstPtr& state_of_task)
{
    whether_arrived_destination_g = 0;
    std::cout << "agv arrive the destination." << std::endl;
}

