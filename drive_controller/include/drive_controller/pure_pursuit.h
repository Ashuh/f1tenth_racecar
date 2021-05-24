#ifndef DRIVE_CONTROLLER_PURE_PURSUIT_H
#define DRIVE_CONTROLLER_PURE_PURSUIT_H

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

namespace f1tenth_racecar
{
namespace control
{
class PurePursuit
{
private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  geometry_msgs::PointStamped look_ahead_point_;
  double look_ahead_point_dist_;
  geometry_msgs::PointStamped arc_center_;
  double arc_radius_;

  double look_ahead_dist_;
  double gain_;

  static double getDist(const geometry_msgs::Point point_1, const geometry_msgs::Point point_2);

  bool isPoseAhead(const nav_msgs::Odometry odom, const geometry_msgs::PoseStamped point);

  geometry_msgs::PointStamped findLookAheadPoint(const nav_msgs::Odometry odom, const nav_msgs::Path path);

public:
  PurePursuit(double look_ahead_dist, double gain);

  double calculateSteeringAngle(const nav_msgs::Odometry odom, const nav_msgs::Path path);

  void getIntermediateResults(geometry_msgs::PointStamped& look_ahead_point, double& look_ahead_point_dist,
                              geometry_msgs::PointStamped& arc_center, double& arc_radius);
};
}  // namespace control
}  // namespace f1tenth_racecar

#endif  // DRIVE_CONTROLLER_PURE_PURSUIT_H
