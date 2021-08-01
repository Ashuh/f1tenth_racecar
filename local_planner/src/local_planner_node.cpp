#include <ros/ros.h>

#include "local_planner/local_planner.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "local_planner");
  LocalPlanner local_planner;
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
