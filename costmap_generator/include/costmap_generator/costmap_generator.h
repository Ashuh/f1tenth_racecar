#ifndef COSTMAP_GENERATOR_COSTMAP_GENERATOR_H
#define COSTMAP_GENERATOR_COSTMAP_GENERATOR_H

#include <string>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <grid_map_ros/grid_map_ros.hpp>

#include "f1tenth_msgs/InflateCostmap.h"

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

  std::map<std::string, double> client_map_;                   // map client to request
  std::map<double, std::pair<std::string, int>> request_map_;  // map request to layer_id and number of clients
  std::map<int, std::string> kernel_map_;                      // map kernel size to layer_id

  static const std::string STATIC_LAYER_;
  static const std::string SCAN_LAYER_;

  double freespace_dist_;  // distance away from obstacles considered to be absolutely free

  void timerCallback(const ros::TimerEvent& timer_event);

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& occ_grid_msg);

  bool
  serviceCallback(ros::ServiceEvent<f1tenth_msgs::InflateCostmapRequest, f1tenth_msgs::InflateCostmapResponse>& event);

  void addRequest(const std::string& client_id, const double radius, const std::string& layer_id);

  void deleteRequest(const std::string& client_id);

  void generateStaticLayer();

  void propagateCosts(cv::Mat& input, cv::Mat& output);

  void inflateMap(const double radius, const std::string& layer_id);

  static std::string generateLayerId(const double radius);
};

#endif  // COSTMAP_GENERATOR_COSTMAP_GENERATOR_H
