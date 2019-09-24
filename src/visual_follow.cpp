#include "visual_follow.h"

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

bool lessThan(ob_point a,ob_point b)
{
    return (a.z < b.z);
}

visualFollow::visualFollow()
{
    pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    sub_ = n_.subscribe<sensor_msgs::PointCloud2> ("/camera/depth/points", 1, &visualFollow::cloudCb,this);

}

visualFollow::~visualFollow()
{
    std::cout << "~visualFollow tid=" << m_tid;
    ROS_INFO("\n");
    if(m_isAlive)
    {
        std::cout << "kill the thread tid=" << m_tid;
        ROS_INFO("\n");
        pthread_kill(m_tid,0);
    }
}

int visualFollow::start()
{
    std::cout << "Start a new thread";
    ROS_INFO("\n");
    if(pthread_create(&m_tid,NULL,start_func,(void*)this) != 0)
    {
        std::cout << "Start a new thread failed!";
        ROS_INFO("\n");
        return -1;
    }

    std::cout << "Start a new thread success! tid =" << m_tid;
    ROS_INFO("\n");
    m_isAlive = true;
    return 0;
}

int visualFollow::quit()
{
    std::cout << "Quit the thread tid=" << m_tid;
    ROS_INFO("\n");
    m_isAlive = false;
    return 0;
}

void visualFollow::run()
{
    ROS_INFO("Run visual follow %d.",m_isAlive);
    while(ros::ok())
    {
        ros::spinOnce();
        if(m_isAlive == false) return;
    }
}

pthread_t visualFollow::getTid()
{
    return m_tid;
}

bool visualFollow::isAlive()
{
    return m_isAlive;
}

void visualFollow::cloudCb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // 声明存储原始数据与滤波后的数据的点云的格式  
  // Container for original & filtered data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; //原始的点云的数据格式
  std::vector<ob_point> obPoint;
  float x = 0,y = 0,z = 0;
  geometry_msgs::Twist volacity;
 
  // Convert to PCL data type。转化为PCL中的点云的数据格式
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  pcl::fromPCLPointCloud2(*cloud,*cloud_data);
  //cout << cloud_data->points.size() << endl;
  for(int i = 0;i < cloud_data->points.size();i++)
  {
    if((cloud_data->points[i].z < 1.2)&&(cloud_data->points[i].z > 0.3))
    {
      obPoint.push_back(ob_point(cloud_data->points[i].x,cloud_data->points[i].y,cloud_data->points[i].z));
    }
  }

  ROS_INFO("The number of point is:%ld",obPoint.size());
  int n = 4000;
  if(obPoint.size() > n)
  {
      sort(obPoint.begin(),obPoint.end(),lessThan);
      for(int i = 0;i < n;i++)
      {
          x += obPoint[i].x;
          y += obPoint[i].y;
          z += obPoint[i].z;
      }
      x = x/n;
      y = y/n;
      z = z/n;
      ROS_INFO("The target is %f,%f,%f",x,y,z);

      if(z > 0.70)
      {
          volacity.linear.x = 0.2;
      }
      else if(z < 0.60)
      {
          volacity.linear.x = -0.2;
      }
      else
      {
          volacity.linear.x = 0.0;
      }

      if(x < -0.15)
      {
          volacity.angular.z = 0.2;
      }
      else if(x > -0.07)
      {
          volacity.angular.z = -0.2;
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

  for(int i = 0;i < obPoint.size();i++)
  {
    obPoint.pop_back();
  }

  //delete cloud_data->points;
  delete cloud;
}

void * visualFollow::start_func(void* arg)
{
    visualFollow *ptr = (visualFollow*) arg;
    ptr->run();
    return NULL;
}

