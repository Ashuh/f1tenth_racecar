#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>

#include "lidar_obstacle_detector/lidar_obstacle_detector.h"

namespace f1tenth_racecar
{
namespace perception
{
LidarObstacleDetection::LidarObstacleDetection()
{
  std::string scan_topic = "scan";
  // std::string obstacle_topic;

  scan_sub_ = nh_.subscribe(scan_topic, 1, &LidarObstacleDetection::scanCallback, this);
  obstacle_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud", 1);
}

void LidarObstacleDetection::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  sensor_msgs::PointCloud2 in_cloud_msg;
  laser_projector_.projectLaser(*scan_msg, in_cloud_msg);

  sensor_msgs::PointCloud2 clustered_msg = point_clusterer_.cluster(in_cloud_msg);
  clustered_msg.header.stamp = ros::Time::now();
  clustered_msg.header.frame_id = "laser";
  obstacle_pub_.publish(clustered_msg);
}
}  // namespace perception
}  // namespace f1tenth_racecar

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "lidat_obstacle_detection");
  f1tenth_racecar::perception::LidarObstacleDetection lidat_obstacle_detection;
  ros::spin();
  return 0;
}
