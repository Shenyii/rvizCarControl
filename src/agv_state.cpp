#include "agv_state.h"

Agv_State::Agv_State(QWidget* parent)
: QWidget(parent)
{
    pub_ = n_.advertise<std_msgs::String>("/agv_state",1);
    sub_ = n_.subscribe<std_msgs::Int8>("/battery_voltage",10,&Agv_State::batteryCallBack,this);

    button_of_power_ = new QPushButton(this);
    button_of_power_->setGeometry(QRect(0,0,130,30));
    button_of_power_->setText("auto navigation");
    button_of_power_->setStyleSheet("background-color:rgb(102,0,102);" "color:rgb(255,255,255);");
    state_of_agv_ = new QTextEdit(this);
    state_of_agv_->setGeometry(QRect(135,0,165,30));
    state_of_agv_->setText("Click to start auto navigation.");

    pose_of_agv_text_ = new QTextEdit(this);
    pose_of_agv_text_->setGeometry(QRect(0,35,300,30));

    button_of_slam_ = new QPushButton(this);
    button_of_slam_->setGeometry(QRect(0,70,130,30));
    button_of_slam_->setText("start slam");
    button_of_slam_->setStyleSheet("background-color:rgb(102,0,102);" "color:rgb(255,255,255);");
    text_of_slam_ = new QTextEdit(this);
    text_of_slam_->setGeometry(QRect(135,70,165,30));
    text_of_slam_->setText("Click to start slam");

    connect(button_of_power_,SIGNAL(clicked(bool)),this,SLOT(clickButtonPower()));
    connect(button_of_slam_,SIGNAL(clicked(bool)),this,SLOT(clickButtonSlam()));
    static int run_times_config_restrict = 0;
    if(run_times_config_restrict == 0)
    {
        run_times_config_restrict = 1;
        position_ = new Tf_Listerner("map","base_footprint");
        QTimer* instance_time = new QTimer(this);
        connect(instance_time,SIGNAL(timeout()),this,SLOT(sendpub()));
        instance_time->start(1000);
        startNewPthread();
    }
}

void Agv_State::clickButtonPower()
{
    std_msgs::String s_state;
    if(button_of_power_->text().toStdString() == "auto navigation")
    {
        s_state.data = "start_navigation";
        pub_.publish(s_state);
        state_of_agv_->setText("AGV can auto navigation.");
        button_of_power_->setText("poweroff");
    }
    else if(button_of_power_->text().toStdString() == "poweroff")
    {
        s_state.data = "poweroff";
        pub_.publish(s_state);
        state_of_agv_->setText("The AGV is stoped");
    }
}

void Agv_State::clickButtonSlam()
{
    std_msgs::String slam_state;
    if(button_of_slam_->text().toStdString() == "start slam")
    {
        button_of_slam_->setText("complete slam");
        text_of_slam_->setText("slam is running, please don't run other task!");
        slam_state.data = "start_slam";
        pub_.publish(slam_state);
    }
    else if(button_of_slam_->text().toStdString() == "complete slam")
    {
        button_of_slam_->setText("end");
        text_of_slam_->setText("Please restart the Rviz!");
        slam_state.data = "complete_slam";
        pub_.publish(slam_state);
    }
}

void Agv_State::sendpub()
{
    char cc[50] = "Agv pose:";
    QString pose_of_agv(cc);
    char pose_s[25];
    sprintf(pose_s,"%f",position_->x());
    pose_of_agv+=pose_s;
    pose_of_agv+=' ';
    pose_of_agv+=',';
    pose_of_agv+=' ';
    sprintf(pose_s,"%f",position_->y());
    pose_of_agv+=pose_s;
    pose_of_agv+=' ';
    pose_of_agv+=',';
    pose_of_agv+=' ';
    if(position_->oz() < 0)
    {
        sprintf(pose_s,"%f",0 - position_->ow());
    }
    else
    {
        sprintf(pose_s,"%f",position_->ow());
    }
    pose_of_agv+=pose_s;
    pose_of_agv_text_->setText(pose_of_agv);
}

void Agv_State::startNewPthread()
{
    ROS_INFO("Start detect battery thread");
    if(pthread_create(&m_tid_,NULL,pthreadFun,(void*)this) != 0)
    {
        std::cout << "Start a new thread failed!";
        ROS_INFO("\n");
    }
}

void* Agv_State::pthreadFun(void* arg)
{
    Agv_State *ptr = (Agv_State*) arg;
    ptr->pthreadProgram();
    return NULL;
}

void Agv_State::pthreadProgram()
{
    //ros::MultiThreadedSpinner spinner(5);
    while(ros::ok())
    {
        ros::Duration(2).sleep();
        ros::spinOnce();
    }
}

void Agv_State::batteryCallBack(const std_msgs::Int8::ConstPtr& voltage)
{
    if(voltage->data < 45)
    {
        state_of_agv_->setTextColor(QColor(255,106,106));
        state_of_agv_->setText("AGV is not enough power,please charge!");
    }
}