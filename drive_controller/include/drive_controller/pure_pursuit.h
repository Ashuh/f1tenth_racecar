#ifndef DRIVE_CONTROLLER_PURE_PURSUIT_H
#define DRIVE_CONTROLLER_PURE_PURSUIT_H

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include "f1tenth_msgs/Trajectory.h"

namespace f1tenth_racecar
{
namespace control
{
class PurePursuit
{
private:
  geometry_msgs::PointStamped look_ahead_point_;
  double look_ahead_point_dist_;
  geometry_msgs::PointStamped arc_center_;
  double arc_radius_;

  double look_ahead_dist_;
  double gain_;

  double calculateSteeringAngle(const geometry_msgs::PointStamped look_ahead_point);

  static double getDist(const geometry_msgs::Point point_1, const geometry_msgs::Point point_2);

  int findLookAheadWaypointId(const nav_msgs::Odometry odom, const f1tenth_msgs::Trajectory trajectory);

public:
  PurePursuit(double look_ahead_dist, double gain);

  ackermann_msgs::AckermannDriveStamped computeDrive(nav_msgs::Odometry odom,
                                                     const f1tenth_msgs::Trajectory trajectory);

  void getIntermediateResults(geometry_msgs::PointStamped& look_ahead_point, double& look_ahead_point_dist,
                              geometry_msgs::PointStamped& arc_center, double& arc_radius);
};
}  // namespace control
}  // namespace f1tenth_racecar

#endif  // DRIVE_CONTROLLER_PURE_PURSUIT_H
