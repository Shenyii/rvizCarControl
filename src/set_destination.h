#ifndef SET_DESTINATION_H
#define SET_DESTINATION_H

#include <QWidget>
#include <stdio.h>
#include <math.h>
#include <QPainter>
#include <QMouseEvent>
#include <QPushButton>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTextEdit>
#include <QLabel>
#include <QTimer>
#include <iostream>
#include <ros/ros.h>
#include <pthread.h>
#include <vector>
#include <string>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include "tf_listerner.h"

#include <math.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

using namespace visualization_msgs;

class Destination_Pose
{
public:
    float x;
    float y;
    float ow;
    float oz;

    Destination_Pose()
    : x(0),y(0),ow(1),oz(0)
    {};

    Destination_Pose(float x1,float y1,float ow1,float oz1)
    {
        x = x1;
        y = y1;
        ow = ow1;
        oz = oz1;
    };
};

class Set_Destination: public QWidget
{
Q_OBJECT
public:
    Set_Destination(QWidget* parent = 0);
    void beginLoopTask();
    virtual QSize sizeHint() const {return QSize(100,100);}

public Q_SLOTS:
    void clickButtonLoopTask();
    void clickButtonCreateDestination();
    void updateLoopTaskText();
    void clickButtonSendDestination();
    void clickButtonAdjustDestination();
    void typeOfOperator();

private:
    int type_of_operator_;
    QString state_and_prompt_text_;
    QString loop_task_text;
    std::vector<int> loop_task_list_;
    int loop_task_ptr_;

    QTextEdit* state_and_prompt_;
    QPushButton* button_loop_task_;
    QTextEdit* text_loop_task_;
    QPushButton* button_create_destination_;
    QPushButton* button_send_destination_;
    QPushButton* button_adjust_destination_;
    
    //void displayDestination(float x,float y,float ow,float oz);

    void PthreadRosServer();
    static void *ThreadRosServer(void * arg);
    void runOtherPthread();
    pthread_t m_tid;
    ros::NodeHandle n_;
    //void make6DofMarker(Destination_Pose destination_pose,int number_name);
    //void displayDestination(std::vector<Destination_Pose> destination_list);
    //void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
};

void make6DofMarker(Destination_Pose destination_pose,int number_name);
void displayDestination(std::vector<Destination_Pose> destination_list);
void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
Marker makeCylinder( InteractiveMarker &msg );
Marker makeArrow( InteractiveMarker &msg );
InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg );

void pthreadSendDestination();
void *sendDestination(void *);
int runModelOfDestination(float *target_pose);

void taskStateCallback(const std_msgs::String::ConstPtr& state_of_task);
#endif