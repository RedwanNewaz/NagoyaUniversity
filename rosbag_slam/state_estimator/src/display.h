#ifndef DISPLAY_H
#define DISPLAY_H
#include <ros/ros.h>
#include <iostream>
#include "visualization_msgs/Marker.h"
#include "sensor_msgs/LaserScan.h"
#include "../include/spline.h"
#include "tf/tf.h"
#define SPLINE_POINTS 100
#define SAMPLE_LINES 100
const int R= 5;

class display
{
public:
    display();
    void robot_state(const float *state);
    void scan(double **points,const float *robot);
    ros::NodeHandle nh;
    ros::Publisher pub_robot,pub_traj;
    ros::Publisher pub_scan;
    visualization_msgs::Marker robot,line_list;
private:
    struct sampling {
        float **data;
        int row,col;
    }sample_lines[SAMPLE_LINES];
protected:
    void sample_init(float x,float y, float yaw);
    void get_samples(double **points, const float *robot);

};

#endif // DISPLAY_H
