#include "drive_widget.h"

namespace rviz_plugin_tutorial
{
    DriveWidget::DriveWidget(QWidget* parent)
    :QWidget(parent),linear_velocity_(0),angular_velocity_(0)
    ,linear_scale_(10),angular_scale_(2)
    {
        // button = new QPushButton(this);
        // button->setGeometry(QRect(5,5,100,30));
        // button->setText("visual follow");

        //connect(button,SIGNAL(clicked(bool)),this,SLOT(clickButton()));
    }

    void DriveWidget::paintEvent(QPaintEvent* event)
    {
        QColor background;
        QColor crosshair;
        if(isEnabled())
        {
            background = Qt::white;
            crosshair = Qt::black;
        }
        else
        {
            background = Qt::lightGray;
            crosshair = Qt::darkGray;
        }

        int w = width();
        int h = height();
        int size = ((w > h) ? h : w) - 1;
        int hpad = (w - size) / 2;
        int vpad = (h - size) / 2;
        QPainter painter(this);
        painter.setBrush(background);
        painter.setPen(crosshair);
        painter.drawRect(QRect(hpad,vpad,size,size));
        painter.drawLine(hpad,height() / 2,hpad + size,height() / 2);
        painter.drawLine(width() / 2,vpad,width() / 2,vpad + size);

        paintArray(hpad,vpad,size,0);
        paintArray(hpad,vpad,size,1);
        paintArray(hpad,vpad,size,2);
        paintArray(hpad,vpad,size,3);

        // button->setGeometry(QRect(5,5,100,30));
        // button->setText("visual follow");
    }

    void DriveWidget::mouseMoveEvent(QMouseEvent* event)
    {
        sendVelocitiesFromMouse(event->x(),event->y(),width(),height());
    }

    void DriveWidget::mousePressEvent(QMouseEvent* event)
    {
        sendVelocitiesFromMouse(event->x(),event->y(),width(),height());
    }

    void DriveWidget::leaveEvent(QEvent* event)
    {
        stop();
    }

    void DriveWidget::mouseReleaseEvent(QMouseEvent* event)
    {
        stop();
    }

    // void DriveWidget::clickButton()
    // {
    //     ROS_INFO("The button has been clicked.");
    //     if(button_state_ == 0)
    //     {
    //         button_state_++;
    //         button->setDown(true);
    //         ROS_INFO("The button is down.");
    //         visual_follow_.start();
    //     }
    //     else
    //     {
    //         button_state_ = 0;
    //         button->setDown(false);
    //         ROS_INFO("The button is up.");
    //         visual_follow_.quit();
    //     }
    // }

    void DriveWidget::sendVelocitiesFromMouse(int x,int y,int width,int height)
    {
        int size = ((width > height) ? height : width);
        int hpad = (width - size) / 2;
        int vpad = (height - size) / 2;
        float slope;
        int x1,y1;
        x1 = x - hpad - size / 2;
        y1 = vpad + size / 2 - y;
        if(x1 == 0)
        {
            linear_velocity_ = 0.2;
            angular_velocity_ = 0.0;
        }
        else
        {
            slope = float(y1) / float(x1);
            if(slope < 0) slope = 0 - slope;
            if((slope > 0) && (slope <= 0.5))
            {
                linear_velocity_ = 0.0;
                angular_velocity_ = -0.3;
            }
            else if((slope > 0.5) && (slope < 2))
            {
                linear_velocity_ = 0.15;
                angular_velocity_ = -0.2;
            }
            else
            {
                linear_velocity_ = 0.3;
                angular_velocity_ = 0.0;
            }
        }
        
        linear_velocity_ = linear_velocity_ * y1 / fabs(y1);
        angular_velocity_ = angular_velocity_ * x1 / fabs(x1);
        if(linear_velocity_ < -0.05) angular_velocity_ = 0 - angular_velocity_;
        Q_EMIT outputVelocity(linear_velocity_,angular_velocity_);
        update();
    }

    void DriveWidget::stop()
    {
        linear_velocity_ = 0;
        angular_velocity_ = 0;
        Q_EMIT outputVelocity(linear_velocity_,angular_velocity_);
        update();
    }

    void DriveWidget::paintArray(int hpad,int vpad,int size,int angle)
    {
        QPainter paint(this);
        int point[14] = {hpad+size,vpad+size/2,hpad+size*5/6,vpad+size/3,hpad+size*5/6,vpad+size*2/3,
               hpad+size*5/6,vpad+size*3/7,hpad+size*5/6,vpad+size*4/7,hpad+size*2/3,vpad+size*4/7,hpad+size*2/3,vpad+size*3/7};
        for(int i = 0;i < angle;i++)
        {
            transform(point,hpad - vpad,hpad + vpad + size);
        }
        QPoint triangle[3] = {QPoint(point[0],point[1]),QPoint(point[2],point[3]),QPoint(point[4],point[5])};
        QPoint square[4] = {QPoint(point[6],point[7]),QPoint(point[8],point[9]),QPoint(point[10],point[11]),QPoint(point[12],point[13])};
        paint.setRenderHint(QPainter::Antialiasing,true);
        QPen pen(QBrush(Qt::green),1,Qt::SolidLine,Qt::SquareCap,Qt::RoundJoin);
        paint.setBrush(QBrush(QColor(102,0,102)));
        paint.setPen(pen);
        paint.drawPolygon(triangle,3);
        paint.drawPolygon(square,4);
    }

    void DriveWidget::transform(int * array,int x,int y)
    {
        int x0,y0;
        for(int i = 0;i < 7;i++)
        {
            x0 = array[2 * i];
            y0 = array[2 * i + 1];
            array[2 * i] = y0 + x;
            array[2 * i + 1] = y - x0;
        }
    }
}
