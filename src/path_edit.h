#ifndef PATH_EDIT_H
#define PATH_EDIT_H

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <ros/ros.h>
#include <math.h>
#include <boost/thread/mutex.hpp>
#include <visualization_msgs/Marker.h>

#include "tf_listerner.h"
#include "move_control.h"
#include "pose.h"

#include <QWidget>
#include <QPushButton>
#include <QTextEdit>
#include <QLabel>
#include <QTimer>

#include <pthread.h>

#include <vector>

using namespace visualization_msgs;

class Path_Edit: public QWidget
{
Q_OBJECT
public:
    Path_Edit(QWidget* parent = 0);

public Q_SLOTS:
    void clickButtonIncreasePath();
    void clickButtonSavePath();
    void clickButtonExecutionPath();
    void clickButtonRemovePath();

private:
    int path_product_flag_;
    QPushButton* button_increase_path_;
    QPushButton* button_save_path_;
    QPushButton* button_execution_path_;
    QTextEdit* text_excution_path_;
    QPushButton* button_remove_path_;
    QTextEdit* text_remove_path_;

    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    interactive_markers::MenuHandler menu_handler_;
    Pose pose_;
    std::vector<Pose> path_of_3point_;
    std::vector<Pose> path_point_;
    pthread_t m_tid;
    int thread_stop_flag_;
    ros::NodeHandle n_;
    ros::Publisher marker_pub_;
    void pthreadRosServer();
    static void* threadRosServer(void* arg);
    void runOtherPthread();
    void make3point(Pose pose,int number_name);
    InteractiveMarkerControl make3Control( InteractiveMarker &msg );
    Marker makeCylind( InteractiveMarker &msg );
    Marker makeArrow1( InteractiveMarker &msg );
    void processFB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void productPathPoint();
    void paintLine();
    void makePath(Pose pose,int number_name,std::string type);
    InteractiveMarkerControl makePointControl( InteractiveMarker &msg );
    Marker makePoint( InteractiveMarker &msg );

    Move_Control* path_tracing_;
};

#endif