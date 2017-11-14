#include "ros/ros.h"
#include "display.h"
#include "file_reader.h"
#include <tf/transform_broadcaster.h>



int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_saver_node");
  ros::NodeHandle nh;
  ros::Rate loop_rate(50);
  display disp;
  file_reader file; float data[5];
  ros::Subscriber map_sub=nh.subscribe("/map",1,&file_reader::save_map,&file);

  //reading file

  file.read_file(argv[1]);
  for(int i=0;i<file.totalLine();i++){
    file.getLine(i,data);
    geometry_msgs::Pose pose;
     pose.position.x= data[0];
     pose.position.y= data[1];
     pose.orientation.x=data[2];
     pose.orientation.y=data[3];
     pose.orientation.z=data[4];
     pose.orientation.w=data[5];
     pose.position.z=1;
     // display robot trajectory
     disp.publishRobot(pose);
     disp.sample_points(pose);


     //sleep
     loop_rate.sleep();
  }
  ros::spin();
return 0;
}


