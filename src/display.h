#ifndef DISPLAY_H
#define DISPLAY_H
#include <iostream>
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PoseStamped.h"
#include "../include/spline.h"
#include <tf/transform_listener.h>
class display
{
public:
  display();
  void spline_trajectory(const geometry_msgs::Pose msg);
  void publishRobot( const geometry_msgs::Pose msg);

private:
  ros::NodeHandle nh;
  ros::Publisher marker_pub;
  visualization_msgs::Marker *line_list,*robot;

};

#endif // DISPLAY_H
