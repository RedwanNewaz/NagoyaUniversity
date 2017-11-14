#include <ros/ros.h>
#include "estimate.h"



int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_estimator_node");
  ros::NodeHandle nh;
  Estimate state;
  state.run(argv[1]);

  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
