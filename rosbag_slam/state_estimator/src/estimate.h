#ifndef ESTIMATE_H
#define ESTIMATE_H
#include "kalman.h"
#include "display.h"
#include <iostream>
#include <eigen3/Eigen/Dense>
#include "file_reader.h"
#include <ros/ros.h>
#include <tf/tf.h>
#include "tf/message_filter.h"
#include <tf/transform_listener.h>
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include <tf/transform_broadcaster.h>
using namespace std;

class LaserScanToPointCloud{

public:

  ros::NodeHandle n_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
  ros::Publisher scan_pub_;

  LaserScanToPointCloud(ros::NodeHandle n) :
    n_(n),
    laser_sub_(n_, "/transformed_scan", 10),
    laser_notifier_(laser_sub_,listener_, "/map", 10)
  {
    laser_notifier_.registerCallback(
      boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
    laser_notifier_.setTolerance(ros::Duration(0.2));
    scan_pub_ = n_.advertise<sensor_msgs::PointCloud2>("/my_cloud",100);
  }

  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
    sensor_msgs::PointCloud2 cloud;
    cloud.header.frame_id="/map";
    cloud.header.stamp=ros::Time::now();
//    ROS_INFO("Transforming. scanmatcher_frame.");
    try
    {

        projector_.transformLaserScanToPointCloud(
          "/map",*scan_in, cloud,listener_);
    }
    catch (tf::TransformException& e)
    {
        std::cout << e.what();
        return;
    }

    // Do something with cloud.

    scan_pub_.publish(cloud);

  }
};

class Estimate
{
public:
    Estimate();
    void run(char *file);
    void getMS();


protected:
   void laser_scan_transform(const sensor_msgs::LaserScan::ConstPtr& msg);
   void hector_slam_pose(const geometry_msgs::PoseStamped::ConstPtr& msg);
   void transform_cloud(const sensor_msgs::PointCloud2::ConstPtr &msg);

private:
    ros::NodeHandle nh;
    ros::Subscriber sub_pose, sub_scan, sub_cloud;
    ros::Subscriber sub_save_path;
    ros::Publisher pub_scan;
    int num_state,num_mes;
    float dt;
    ros::Time stamp;
    KalmanFilter *kf;
    display *disp;
    LaserScanToPointCloud  *lspc;
    tf::TransformBroadcaster br;
    file_reader *file;
};

#endif // ESTIMATE_H
