#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Polygon.h>
#include <visualization_msgs/MarkerArray.h>

#include <laser_geometry/laser_geometry.h>

#include "lidar_obstacle_detector/lidar_obstacle_detector.h"

namespace f1tenth_racecar
{
namespace perception
{
LidarObstacleDetection::LidarObstacleDetection()
{
  std::string scan_topic = "scan";

  scan_sub_ = nh_.subscribe(scan_topic, 1, &LidarObstacleDetection::scanCallback, this);
  clustered_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud", 1);
  obstacle_pub_ = nh_.advertise<f1tenth_msgs::ObstacleArray>("obstacles", 1);
  obstacle_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("obstacles_visualization", 1);
}

void LidarObstacleDetection::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  sensor_msgs::PointCloud2 in_cloud_msg;
  laser_projector_.projectLaser(*scan_msg, in_cloud_msg);

  sensor_msgs::PointCloud2 clusters_msg;
  f1tenth_msgs::ObstacleArray obstacles;

  point_clusterer_.cluster(in_cloud_msg, clusters_msg, obstacles);
  clustered_cloud_pub_.publish(clusters_msg);
  obstacle_pub_.publish(obstacles);
  visualizeObstacles(obstacles);
}

void LidarObstacleDetection::visualizeObstacles(const f1tenth_msgs::ObstacleArray obstacles)
{
  visualization_msgs::MarkerArray obstacle_markers;

  for (int i = 0; i < obstacles.obstacles.size(); ++i)
  {
    f1tenth_msgs::Obstacle obstacle = obstacles.obstacles.at(i);
    visualization_msgs::Marker obstacle_marker;

    for (int j = 0; j < obstacle.footprint.points.size() + 1; j++)
    {
      geometry_msgs::Point p;
      p.x = obstacle.footprint.points.at(j % obstacle.footprint.points.size()).x;
      p.y = obstacle.footprint.points.at(j % obstacle.footprint.points.size()).y;
      obstacle_marker.points.push_back(p);
    }

    obstacle_marker.action = visualization_msgs::Marker::ADD;
    obstacle_marker.type = visualization_msgs::Marker::LINE_STRIP;
    obstacle_marker.id = i;
    obstacle_marker.lifetime = ros::Duration(0.1);
    obstacle_marker.header.frame_id = "laser";

    obstacle_marker.scale.x = 0.05;
    obstacle_marker.scale.y = 0.05;
    obstacle_marker.scale.z = 0.05;

    obstacle_marker.color.r = 0;
    obstacle_marker.color.g = 0;
    obstacle_marker.color.b = 1;
    obstacle_marker.color.a = 1;

    obstacle_markers.markers.push_back(obstacle_marker);
  }
  obstacle_viz_pub_.publish(obstacle_markers);
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
