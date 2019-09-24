#include "set_initial_position.h"

Set_Initial_Position::Set_Initial_Position(QWidget* parent)
: QWidget(parent),x_(0),y_(0),oz_(0),ow_(1)
{
    pub_ = n_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",1);

    button_ = new QPushButton("initial position");

    QHBoxLayout* position_layout = new QHBoxLayout;
    position_layout->addWidget(button_);
    output_home_position_editor_ = new QLineEdit;
    position_layout->addWidget(output_home_position_editor_);
    QHBoxLayout* layout = new QHBoxLayout;
    layout->addLayout(position_layout);
    setLayout(layout);

    connect(button_,SIGNAL(clicked(bool)),this,SLOT(clickButton()));
    connect(output_home_position_editor_,SIGNAL(editingFinished()),this,SLOT(update_home_position()));

    ROS_INFO("initial position(%f,%f,%f,%f).",x_,y_,ow_,oz_);
}

void Set_Initial_Position::initial_position()
{
    //ROS_INFO("Will set the initial position(%f,%f,%f,%f).",x_,y_,ow_,oz_);
    //update_home_position();
    geometry_msgs::PoseWithCovarianceStamped msg_poseinit;
    msg_poseinit.header.frame_id = "map";
    msg_poseinit.header.stamp = ros::Time::now();
    msg_poseinit.pose.pose.position.x = x_;
    msg_poseinit.pose.pose.position.y = y_;
    msg_poseinit.pose.pose.orientation.z = oz_;
    msg_poseinit.pose.pose.orientation.w = ow_;
    pub_.publish(msg_poseinit);
    ros::Duration(1.0).sleep();
    pub_.publish(msg_poseinit);
    
    ROS_INFO("Had set the initial position(%f,%f,%f,%f).",x_,y_,ow_,oz_);
}

QString Set_Initial_Position::get_home_position()
{
    return output_home_position_;
}

void Set_Initial_Position::set_output_home_position_editor(const QString& home_position)
{
    output_home_position_editor_->setText(home_position);
    output_home_position_ = home_position;
    ROS_INFO("load the home position(%s).",output_home_position_.toStdString().c_str());
}

void Set_Initial_Position::clickButton()
{
    //ROS_INFO("Set the initial position.");
    initial_position();
}

void Set_Initial_Position::update_home_position()
{
    output_home_position_editor_->setText(output_home_position_);
    output_home_position_ = output_home_position_editor_->text();
    ROS_INFO("%s",output_home_position_.toStdString().c_str());
    string_to_num(output_home_position_.toStdString(),',');
}

void Set_Initial_Position::string_to_num(std::string string_of_position,char delim)
{
    std::string one_of_string[3];
    int n = 0;
    for(int i = 0;i < string_of_position.size();i++)
    {
        if(string_of_position[i] != delim)
        {
            one_of_string[n]+=string_of_position[i];
        }
        else
        {
            n++;
        }
    }

    x_ = atof(one_of_string[0].c_str());
    y_ = atof(one_of_string[1].c_str());
    ow_ = cos(atof(one_of_string[2].c_str()) / 2);
    oz_ = sqrt(1 - ow_ * ow_);

    ROS_INFO("(%f)",x_);
}
