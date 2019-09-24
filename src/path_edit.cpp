#include "path_edit.h"

Path_Edit::Path_Edit(QWidget* parent)
: QWidget(parent),path_product_flag_(0),thread_stop_flag_(0)
{
    button_increase_path_ = new QPushButton(this);
    button_increase_path_->setGeometry(QRect(0,0,130,30));
    button_increase_path_->setText("generate path");
    button_increase_path_->setStyleSheet("background-color:rgb(102,0,102);" "color:rgb(255,255,255);");

    button_save_path_ = new QPushButton(this);
    button_save_path_->setGeometry(QRect(170,0,130,30));
    button_save_path_->setText("delete path");
    button_save_path_->setStyleSheet("background-color:rgb(102,0,102);" "color:rgb(255,255,255);");

    // button_execution_path_ = new QPushButton(this);
    // button_execution_path_->setGeometry(QRect(0,35,130,30));
    // button_execution_path_->setText("execution path");
    // text_excution_path_ = new QTextEdit(this);
    // text_excution_path_->setGeometry(QRect(135,35,165,30));
    // text_excution_path_->setText("Number of path!");

    // button_remove_path_ = new QPushButton(this);
    // button_remove_path_->setGeometry(QRect(0,70,130,30));
    // button_remove_path_->setText("remove path");
    // text_remove_path_ = new QTextEdit(this);
    // text_remove_path_->setGeometry(QRect(135,70,165,30));
    // text_remove_path_->setText("Number of path!");

    connect(button_increase_path_,SIGNAL(clicked(bool)),this,SLOT(clickButtonIncreasePath()));
    connect(button_save_path_,SIGNAL(clicked(bool)),this,SLOT(clickButtonSavePath()));
    //connect(button_execution_path_,SIGNAL(clicked(bool)),this,SLOT(clickButtonExecutionPath()));
    //connect(button_remove_path_,SIGNAL(clicked(bool)),this,SLOT(clickButtonRemovePath()));

    static int run_limit = 0;
    if(run_limit == 0)
    {
        pthreadRosServer();
        run_limit++;

        path_tracing_ = new Move_Control;
    }
}

void Path_Edit::clickButtonIncreasePath()
{
    if(path_product_flag_ == 0)
    {
        path_product_flag_ = 1;
        thread_stop_flag_ = 0;
        path_of_3point_.clear();
        for(int i = 0;i < 3;i++)
        {
            Pose pose_init(i,0,0,1);
            path_of_3point_.push_back(pose_init);
            make3point(pose_init,i);
            server_->applyChanges();
        }
    }
}

void Path_Edit::clickButtonSavePath()
{
    thread_stop_flag_ = 1;
    if(path_product_flag_ == 0)
    {
        ROS_INFO("There are not path to save!Please click the button of increase path.");
    }
    else if(path_product_flag_ == 1)
    {
        //save the path
        server_.reset();
        server_.reset(new interactive_markers::InteractiveMarkerServer("path_point","",false));
        path_product_flag_ = 0;
    }
}

// void Path_Edit::clickButtonExecutionPath()
// {
//     int path_number;
//     path_number = atoi(text_excution_path_->toPlainText().toStdString().c_str());
//     ROS_INFO("%d.",path_number);
//     if((path_number > 0) && (path_number < 20))
//     {
//         //excute path
//         text_excution_path_->setText("Execution the path.Don't run other program!!!");
//     }
//     else
//     {
//         text_excution_path_->setText("Please input a number of path.");
//     }
// }

// void Path_Edit::clickButtonRemovePath()
// {
//     int path_number;
//     path_number = atoi(text_remove_path_->toPlainText().toStdString().c_str());
//     ROS_INFO("%d.",path_number);
//     if((path_number > 0) && (path_number < 20))
//     {
//         //remove path
//     }
//     else
//     {
//         text_remove_path_->setText("Please input a number of path.");
//     }
// }

void Path_Edit::pthreadRosServer()
{
    if(pthread_create(&m_tid,NULL,threadRosServer,(void*)this) != 0)
    {
        ROS_INFO("Start increase path thread failed!");
        return; 
    }
}

void* Path_Edit::threadRosServer(void* arg)
{
    Path_Edit *ptr = (Path_Edit*) arg;
    ptr->runOtherPthread();
    return NULL;
}

void Path_Edit::runOtherPthread()
{
    server_.reset(new interactive_markers::InteractiveMarkerServer("path_point","",false));
    menu_handler_.insert("Run from here.",boost::bind(&Path_Edit::processFB,this,_1));

    while(ros::ok())
    {
        ros::spinOnce();
        ros::Duration(1).sleep();
    }
    server_.reset();
}

void Path_Edit::make3point(Pose pose,int number_name)
{
    tf::Vector3 position;
    position = tf::Vector3(pose.x,pose.y,0.05);
    bool fixed = false;
    unsigned int interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
    bool show_6dof = true;

    InteractiveMarker int_marker;
    int_marker.header.frame_id = "map";
    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.pose.orientation.x = 0;
    int_marker.pose.orientation.y = 0;
    int_marker.pose.orientation.z = pose.oz;
    int_marker.pose.orientation.w = pose.ow;
    int_marker.scale = 1;
    int_marker.description = "number_name:";
    char name_s[25];
    sprintf(name_s,"%d",number_name);
    int_marker.name = name_s;
    int_marker.description += name_s;

    // insert a box
    make3Control(int_marker);
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
        control.name = "move_x";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        orien = tf::Quaternion(0.0, 0.0, 1.0, 1.0);
        orien.normalize();
        tf::quaternionTFToMsg(orien, control.orientation);
        control.name = "move_y";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        orien = tf::Quaternion(0.0, 1.0, 0.0, 1.0);
        orien.normalize();
        tf::quaternionTFToMsg(orien, control.orientation);
        control.name = "rotate_z";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
    }

    server_->insert(int_marker);
    server_->setCallback(int_marker.name, boost::bind(&Path_Edit::processFB,this,_1));
    if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
        menu_handler_.apply( *server_, int_marker.name );
    server_->applyChanges();
}

InteractiveMarkerControl Path_Edit::make3Control( InteractiveMarker &msg )
{
    InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back( makeCylind(msg) );
    control.markers.push_back( makeArrow1(msg) );
    msg.controls.push_back( control );

    return msg.controls.back();
}

Marker Path_Edit::makeCylind( InteractiveMarker &msg )
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

Marker Path_Edit::makeArrow1( InteractiveMarker &msg )
{
    Marker marker;

    marker.type = Marker::MESH_RESOURCE;
    marker.mesh_resource = "package://rvizCarControl/media/tringle2.dae";
    marker.pose.position.x = 0.2;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.color.r = 0.9;
    marker.color.g = 0.2;
    marker.color.b = 0.2;
    marker.color.a = 1.0;

    return marker;
}

void Path_Edit::processFB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    int point_id;
    point_id = atoi(feedback->marker_name.c_str());
    Pose change_pose;
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
        {
            if((point_id == 0) && (feedback->menu_entry_id == 1))
            {
                ROS_INFO("start run from 0");
                path_tracing_->getPath(path_point_);
            }
            else if((point_id == 2) && (feedback->menu_entry_id == 1))
            {
                ROS_INFO("start run from 2");
                std::vector<Pose> reverse_path_point;
                for(int i = 0;i < path_point_.size();i++)
                {
                    reverse_path_point.push_back(path_point_[path_point_.size() - i - 1]);
                }
                path_tracing_->getPath(reverse_path_point);
                reverse_path_point.clear();
            }
        }
        break;

        case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
            //ROS_INFO("update the %d pose",destination_id);
            path_of_3point_.erase(path_of_3point_.begin() + point_id,path_of_3point_.begin() + point_id + 1);
            path_of_3point_.insert(path_of_3point_.begin() + point_id,change_pose);
            break;

        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
            break;

        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
            //displayDestination(destination_list_g);
            productPathPoint();
            break;
    }
    server_->applyChanges();
}

void Path_Edit::productPathPoint()
{
    path_point_.clear();
    tf::Vector3 v1(path_of_3point_[0].x - path_of_3point_[1].x,path_of_3point_[0].y - path_of_3point_[1].y,0);
    tf::Vector3 v2(path_of_3point_[2].x - path_of_3point_[1].x,path_of_3point_[2].y - path_of_3point_[1].y,0);
    float angle_v1_v2 = tfAngle(v1,v2);
    float slope_1 = v1.getY() / v1.getX();
    float slope_2 = v2.getY() / v2.getX();
    Pose transition_point_1;
    Pose transition_point_2;

    if(fabs(slope_1 - slope_2) < 0.01)
    {
        ROS_INFO("paint a line-.");
    }
    else if((fabs(slope_1) > 9999) && (fabs(slope_2) > 9999))
    {
        ROS_INFO("paint a line1.");
    }
    else if(fabs(slope_1) > 9999)
    {
        ROS_INFO("line 1 is vertical.");
    }
    else if(fabs(slope_2) > 9999)
    {
        ROS_INFO("line 3 is vertical.");
    }
    else
    {
        transition_point_1.x = path_of_3point_[1].x + v1.getX() / v1.length() / tan(angle_v1_v2 / 2);
        transition_point_1.y = path_of_3point_[1].y + v1.getY() / v1.length() / tan(angle_v1_v2 / 2);
        transition_point_2.x = path_of_3point_[1].x + v2.getX() / v2.length() / tan(angle_v1_v2 / 2);
        transition_point_2.y = path_of_3point_[1].y + v2.getY() / v2.length() / tan(angle_v1_v2 / 2);
        Pose one_point;
        one_point.x = path_of_3point_[0].x;
        one_point.y = path_of_3point_[0].y;
        one_point.oz = path_of_3point_[0].oz;
        one_point.ow = path_of_3point_[0].ow;
        path_point_.push_back(one_point);

        Pose center_of_circle;
        int path_segment = 0;
        center_of_circle.x = (slope_2*(transition_point_1.x+slope_1*transition_point_1.y)-slope_1*(transition_point_2.x+slope_2*transition_point_2.y))/(slope_2-slope_1);
        center_of_circle.y = (transition_point_2.x+slope_2*transition_point_2.y-transition_point_1.x-slope_1*transition_point_1.y)/(slope_2-slope_1);

        for(int i = 1;i < sqrt((transition_point_1.x-path_of_3point_[0].x)*(transition_point_1.x-path_of_3point_[0].x)+(transition_point_1.y-path_of_3point_[0].y)*(transition_point_1.y-path_of_3point_[0].y))*10;i++)
        {
            //ROS_INFO("paint the first line.");
            one_point.x = path_of_3point_[0].x - v1.getX() / v1.length() * 0.1 * i;
            one_point.y = path_of_3point_[0].y - v1.getY() / v1.length() * 0.1 * i;
            one_point.oz = 0;
            one_point.ow = 1;
            path_point_.push_back(one_point);
        }

        float start_angle;
        tf::Vector3 start_vector(transition_point_1.x-center_of_circle.x,transition_point_1.y-center_of_circle.y,0);
        start_angle = tfAngle(start_vector,tf::Vector3(1,0,0));
        if(start_vector.getY() < 0)
        {
            start_angle = 0 - start_angle;
        }
        float x_end = center_of_circle.x + cos(start_angle + 3.14159265 - angle_v1_v2);
        float y_end = center_of_circle.y + sin(start_angle + 3.14159265 - angle_v1_v2);
        if((x_end-transition_point_2.x)*(x_end-transition_point_2.x)+(y_end-transition_point_2.y)*(y_end-transition_point_2.y) < 0.01)
        {
            for(int i = 0;i < (3.14159265 - angle_v1_v2) * 10;i++)
            {
                //ROS_INFO("paint the secend line.");
                one_point.x = center_of_circle.x + cos(start_angle + 0.1 * i);
                one_point.y = center_of_circle.y + sin(start_angle + 0.1 * i);
                one_point.oz = 0;
                one_point.ow = 1;
                path_point_.push_back(one_point);
            }
        }
        else
        {
            for(int i = 0;i < (3.14159265 - angle_v1_v2) * 10;i++)
            {
                //ROS_INFO("paint the secend line.");
                one_point.x = center_of_circle.x + cos(start_angle - 0.1 * i);
                one_point.y = center_of_circle.y + sin(start_angle - 0.1 * i);
                one_point.oz = 0;
                one_point.ow = 1;
                path_point_.push_back(one_point);
            }
        }
 
        for(int i = 0;i < sqrt((transition_point_2.x-path_of_3point_[2].x)*(transition_point_2.x-path_of_3point_[2].x)+(transition_point_2.y-path_of_3point_[2].y)*(transition_point_2.y-path_of_3point_[2].y))*10;i++)
        {
            //ROS_INFO("paint the third line.");
            one_point.x = transition_point_2.x + v2.getX() / v2.length() * 0.1 * i;
            one_point.y = transition_point_2.y + v2.getY() / v2.length() * 0.1 * i;
            one_point.oz = 0;
            one_point.ow = 1;
            path_point_.push_back(one_point);
        }

        one_point.x = path_of_3point_[2].x;
        one_point.y = path_of_3point_[2].y;
        one_point.oz = path_of_3point_[2].oz;
        one_point.ow = path_of_3point_[2].ow;
        path_point_.push_back(one_point);
    }
    paintLine();
}

void Path_Edit::paintLine()
{
    server_.reset();
    server_.reset(new interactive_markers::InteractiveMarkerServer("path_point","",false));
    for(int i = 0;i < path_of_3point_.size();i++)
    {
        make3point(path_of_3point_[i],i);
    }

    for(int i = 0;i < path_point_.size();i++)
    {
        makePath(path_point_[i],i,"path");
    }
}

void Path_Edit::makePath(Pose pose,int number_name,std::string type)
{
    tf::Vector3 position;
    position = tf::Vector3(pose.x,pose.y,0.05);
    bool fixed = false;
    unsigned int interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
    bool show_6dof = true;

    InteractiveMarker int_marker;
    int_marker.header.frame_id = "map";
    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.pose.orientation.x = 0;
    int_marker.pose.orientation.y = 0;
    int_marker.pose.orientation.z = pose.oz;
    int_marker.pose.orientation.w = pose.ow;
    int_marker.scale = 1;
    //int_marker.description = "number_name:";
    char name_s[25];
    sprintf(name_s,"%d",number_name);
    int_marker.name = name_s;
    int_marker.name += type;
    int_marker.description += name_s;

    // insert a box
    makePointControl(int_marker);
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
    {}

    server_->insert(int_marker);
    //server_->setCallback(int_marker.name, boost::bind(&Path_Edit::processFB,this,_1));
    //if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
    //    menu_handler.apply( *server_, int_marker.name );
    server_->applyChanges();
}

InteractiveMarkerControl Path_Edit::makePointControl( InteractiveMarker &msg )
{
    InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back( makePoint(msg) );
    msg.controls.push_back( control );

    return msg.controls.back();
}

Marker Path_Edit::makePoint( InteractiveMarker &msg )
{
    Marker marker;

    marker.type = Marker::CYLINDER;
    marker.scale.x = msg.scale * 0.05;
    marker.scale.y = msg.scale * 0.05;
    marker.scale.z = msg.scale * 0.05;
    marker.color.r = 0.1;
    marker.color.g = 0.9;
    marker.color.b = 0.1;
    marker.color.a = 1.0;

    return marker;
}

