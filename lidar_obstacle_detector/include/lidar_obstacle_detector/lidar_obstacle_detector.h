#ifndef LIDAR_OBSTACLE_DETECTOR_LIDAR_OBSTACLE_DETECTOR_H
#define LIDAR_OBSTACLE_DETECTOR_LIDAR_OBSTACLE_DETECTOR_H

#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>

#include "lidar_obstacle_detector/pcl_euclidean_clusterer.h"

namespace f1tenth_racecar
{
namespace perception
{
class LidarObstacleDetection
{
private:
  ros::NodeHandle nh_;

  // Subscribers
  ros::Subscriber scan_sub_;

  // Publishers
  ros::Publisher clustered_cloud_pub_;
  ros::Publisher obstacle_pub_;
  ros::Publisher obstacle_viz_pub_;

  laser_geometry::LaserProjection laser_projector_;

  PCLEuclideanClusterer point_clusterer_;

  // std::string obstacle_topic_;
  // std::string obstacle_viz_topic_;

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
  void visualizeObstacles(std::vector<geometry_msgs::Polygon> obstacles);

public:
  LidarObstacleDetection();
};
}  // namespace perception
}  // namespace f1tenth_racecar

#endif  // LIDAR_OBSTACLE_DETECTOR_LIDAR_OBSTACLE_DETECTOR_H
