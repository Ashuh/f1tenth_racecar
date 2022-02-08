#include <ros/ros.h>

#include "f1tenth_ll2_global_planner/global_planner.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "global_planner");
  GlobalPlanner global_planner;
  ros::spin();
  return 0;
}
