#ifndef DRIVE_WIDGET_H
#define DRIVE_WIDGET_H

#include <QWidget>
#include <stdio.h>
#include <math.h>
#include <QPainter>
#include <QMouseEvent>
#include <QPushButton>
#include <iostream>
#include <ros/ros.h>
#include <pthread.h>
#include "visual_follow.h"
#include "set_initial_position.h"

//using namespace std;

namespace rviz_plugin_tutorial
{
    class DriveWidget: public QWidget
    {
    Q_OBJECT
    public:
        DriveWidget(QWidget* parent = 0);
        virtual void paintEvent(QPaintEvent* event);
        virtual void mouseMoveEvent(QMouseEvent* event);
        virtual void mousePressEvent(QMouseEvent* event);
        virtual void mouseReleaseEvent(QMouseEvent* event);
        virtual void leaveEvent(QEvent* event);
        virtual QSize sizeHint() const {return QSize(100,100);}

    public Q_SLOTS:
        void clickButton();

    Q_SIGNALS:
        void outputVelocity(float linear,float angular);

    protected:
        void sendVelocitiesFromMouse(int x,int y,int width,int height);
        void stop();
        void paintArray(int hpad,int vpad,int size,int angle);
        void transform(int * array,int x,int y);

        float linear_velocity_;
        float angular_velocity_;
        float linear_scale_;
        float angular_scale_;

        QPushButton *button;
        int button_state_;

        visualFollow visual_follow_;
    };
}
#endif