#ifndef DISPLAY_H
#define DISPLAY_H
#include <iostream>
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PoseStamped.h"
#include "../include/spline.h"
#include <tf/transform_listener.h>
#define SPLINE_POINTS 100
#define SAMPLE_LINES 15
#define R 5
class display
{
public:
  display();
  void publishRobot( const geometry_msgs::Pose msg);
  void sample_points(const geometry_msgs::Pose msg);

private:
  ros::NodeHandle nh;
  ros::Publisher marker_pub;
  visualization_msgs::Marker *line_list,*robot;

  struct sampling {
      float **data;
      int row,col;
  }sample_lines[SAMPLE_LINES];
protected:
  void sample_init(float x,float y, float yaw);
  void get_rotated_points(float x,float y,float *x1,float *y1,float theta, int n);

};

#endif // DISPLAY_H
