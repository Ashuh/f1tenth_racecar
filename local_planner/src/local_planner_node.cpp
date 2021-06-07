#include <ros/ros.h>

#include "local_planner/local_planner.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "local_planner");
  LocalPlanner local_planner;
  ros::spin();
  return 0;
}
