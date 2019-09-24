#ifndef AGV_STATE_H
#define AGV_STATE_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <QPushButton>
#include <QTextEdit>
#include <QTimer>
#include <stdlib.h>
#include <sstream>
#include "tf_listerner.h"
#include <pthread.h>
#include <iostream>
#include <signal.h>

class Agv_State: public QWidget
{
Q_OBJECT
public:
    Agv_State(QWidget* parent = 0);
    virtual QSize sizeHint() const {return QSize(30,100);}

public Q_SLOTS:
    void clickButtonPower();
    void sendpub();
    void clickButtonSlam();

private:
    QPushButton* button_of_power_;
    QTextEdit* state_of_agv_;
    QTextEdit* pose_of_agv_text_;
    QPushButton* button_of_slam_;
    QTextEdit* text_of_slam_;

    ros::NodeHandle n_;
    ros::Publisher pub_;

    Tf_Listerner* position_;

    void startNewPthread();
    static void* pthreadFun(void* arg);
    void pthreadProgram();
    pthread_t m_tid_;
    ros::Subscriber sub_;
    void batteryCallBack(const std_msgs::Int8::ConstPtr& voltage);
};

#endif