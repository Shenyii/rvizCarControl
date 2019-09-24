#ifndef SET_INITIAL_POSITION_H
#define SET_INITIAL_POSITION_H

#include <QWidget>
#include <QPushButton>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <rviz/panel.h>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <string>
#include <vector>

class Set_Initial_Position:public QWidget
{
Q_OBJECT
public:
    Set_Initial_Position(QWidget* parent = 0);
    void initial_position();
    virtual QSize sizeHint() const {return QSize(150,80);}
    QString get_home_position();
    void set_output_home_position_editor(const QString& home_position);

public Q_SLOTS:
    void clickButton();

//protected Q_SLOTS:
    void update_home_position();

//protected:
private:
    QPushButton *button_;
    ros::NodeHandle n_;
    ros::Publisher pub_;
    float x_;
    float y_;
    float ow_;
    float oz_;
    QLineEdit* output_home_position_editor_;
    QString output_home_position_;
    std::vector<std::string> home_position_;
    void string_to_num(std::string string_of_position,char delim);
};
#endif