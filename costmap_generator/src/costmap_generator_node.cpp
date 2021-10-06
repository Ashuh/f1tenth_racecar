#include <ros/ros.h>

#include "costmap_generator/costmap_generator.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "cost_map_generator");
  CostmapGenerator costmap_generator;
  ros::spin();
  return 0;
}
