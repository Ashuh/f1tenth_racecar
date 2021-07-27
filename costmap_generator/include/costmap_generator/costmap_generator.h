#ifndef COSTMAP_GENERATOR_COSTMAP_GENERATOR_H
#define COSTMAP_GENERATOR_COSTMAP_GENERATOR_H

#include <string>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <tf2_ros/transform_listener.h>

class CostmapGenerator
{
private:
  ros::NodeHandle nh_;
  ros::Timer timer_;

  ros::Subscriber map_sub_;
  ros::Subscriber scan_sub_;
  ros::Publisher cost_map_pub_;

  grid_map::GridMap global_map_;
  grid_map::GridMap local_map_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

public:
  CostmapGenerator();

  void timerCallback(const ros::TimerEvent& timer_event);

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& occ_grid_msg);

  void generateStaticLayer();

  void inflateOccupiedCells();
};

#endif  // COSTMAP_GENERATOR_COSTMAP_GENERATOR_H
