#include "object_follow.h"

class ob_point
{
  //private:
  public:
      float x;
      float y;
      float z;

  //public:
      ob_point()
      :x(0.0),y(0.0),z(0.0)
      {}

      ob_point(float tx,float ty,float tz)
      :x(tx),y(ty),z(tz)
      {}

      void display()
      {
        std::cout << "x=" << x << "y=" << y << "z=" << z; 
        ROS_INFO("\n");
      }
};

bool lessThan2(ob_point a,ob_point b)
{
    return (a.z < b.z);
}

Object_Follow::Object_Follow(QWidget* parent)
:QWidget(parent),stop_flag_(5)
{
    pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    sub_ = n_.subscribe<sensor_msgs::LaserScan> ("/scan", 1, &Object_Follow::cloudCb,this);

    button_follow_ = new QPushButton(this);
    button_follow_->setGeometry(QRect(0,35,130,30));
    button_follow_->setText("start object follow");
    button_follow_->setStyleSheet("background-color:rgb(102,0,102);" "color:rgb(255,255,255);");
    text_follow_ = new QTextEdit(this);
    text_follow_->setGeometry(QRect(135,35,165,30));
    text_follow_->setText("click button to start.");

    connect(button_follow_,SIGNAL(clicked(bool)),this,SLOT(clickButtonFollow()));
}

Object_Follow::~Object_Follow()
{
    std::cout << "~Object_Follow tid=" << m_tid;
    ROS_INFO("\n");
    if(!m_isAlive)
    {
        std::cout << "kill the thread tid=" << m_tid;
        ROS_INFO("\n");
        pthread_kill(m_tid,0);
    }
}

int Object_Follow::start()
{
    ROS_INFO("Start object follow thread");
    m_isAlive = true;
    if(pthread_create(&m_tid,NULL,start_func,(void*)this) != 0)
    {
        std::cout << "Start a new thread failed!";
        ROS_INFO("\n");
        return -1;
    }
    return 0;
}

int Object_Follow::quit()
{
    ROS_INFO("Quit the object follow thread");
    m_isAlive = false;
    return 0;
}

void Object_Follow::run()
{
    ROS_INFO("Run visual follow %d.",m_isAlive);
    while(ros::ok())
    {
        ros::spinOnce();
        if(m_isAlive == false) return;
    }
}

pthread_t Object_Follow::getTid()
{
    return m_tid;
}

bool Object_Follow::isAlive()
{
    return m_isAlive;
}

void Object_Follow::clickButtonFollow()
{
    if(button_follow_->text().toStdString() == "start object follow")
    {
        button_follow_->setText("stop object follow");
        text_follow_->setText("The AGV is following the object,please don,t run other program!");
        start();
    }
    else if(button_follow_->text().toStdString() == "stop object follow")
    {
        button_follow_->setText("start object follow");
        text_follow_->setText("Follow program is stoped!");
        quit();
    }
}

void Object_Follow::cloudCb(const sensor_msgs::LaserScanConstPtr& cloud_msg)
{
  geometry_msgs::Twist volacity;
  if(button_follow_->text().toStdString() == "start object follow")
  {
      if(stop_flag_ == 0)
      {
          volacity.linear.x = 0.0;
          volacity.angular.z = 0.0;
          pub_.publish(volacity);
      }
      stop_flag_ = 5;
      return;
  }
  else
  {
      stop_flag_ = 0;
  }

  std::vector<ob_point> obPoint;
  int ranges = cloud_msg->ranges.size();
  for(int i = 250;i < 750;i++)
  {
      if((cloud_msg->ranges[i] < 1.5) && (cloud_msg->ranges[i] > 0.3))
      {
          ob_point one_point;
          one_point.z = cloud_msg->ranges[i];
          one_point.x = 3.14159265 / 1000 * i - 0.5 * 3.14159265;
          obPoint.push_back(one_point);
      }
  }
  int n = 25;
  if(obPoint.size() > n)
  {
      float x;
      float y;
      float z;
      sort(obPoint.begin(),obPoint.end(),lessThan2);
      for(int i = 0;i < n;i++)
      {
          x += obPoint[i].x;
          y += obPoint[i].y;
          z += obPoint[i].z;
      }
      x = x/n;
      y = y/n;
      z = z/n;
      ROS_INFO("The target is %f,%f",x,z);

      if(z > 0.60)
      {
          volacity.linear.x = 0.2;
      }
      else if(z < 0.50)
      {
          volacity.linear.x = -0.2;
      }
      else
      {
          volacity.linear.x = 0.0;
      }

      if(x < -0.1)
      {
          volacity.angular.z = -0.3;
      }
      else if(x > 0.1)
      {
          volacity.angular.z = 0.3;
      }
      else
      {
          volacity.angular.z = 0.0;
      }

      pub_.publish(volacity);
  }
  else
  {
      ROS_INFO("There are not target or you are too close to robot !");
      volacity.linear.x = 0.0;
      volacity.angular.z = 0.0;
      pub_.publish(volacity);
  }

  obPoint.clear();
}

void * Object_Follow::start_func(void* arg)
{
    Object_Follow *ptr = (Object_Follow*) arg;
    ptr->run();
    return NULL;
}

