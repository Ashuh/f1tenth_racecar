#ifndef COSTMAP_GENERATOR_COSTMAP_GENERATOR_H
#define COSTMAP_GENERATOR_COSTMAP_GENERATOR_H

#include <string>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <grid_map_ros/grid_map_ros.hpp>

class CostmapGenerator
{
public:
  CostmapGenerator();

private:
  ros::NodeHandle nh_;
  ros::Timer timer_;

  ros::Subscriber map_sub_;
  ros::Subscriber scan_sub_;
  ros::Publisher costmap_pub_;

  ros::ServiceServer server_;

  grid_map::GridMap global_map_;
  grid_map::GridMap local_map_;

  double inflation_radius_;

  static const std::string STATIC_LAYER_;
  static const std::string SCAN_LAYER_;
  static const std::string INFLATION_LAYER_;

  double freespace_dist_;  // distance away from obstacles considered to be absolutely free

  void timerCallback(const ros::TimerEvent& timer_event);

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& occ_grid_msg);

  void generateStaticLayer();

  void propagateCosts(cv::Mat& input, cv::Mat& output);

  void inflateMap(const double radius, const std::string& layer_id);
};

#endif  // COSTMAP_GENERATOR_COSTMAP_GENERATOR_H
