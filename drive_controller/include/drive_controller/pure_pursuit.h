#ifndef DRIVE_CONTROLLER_PURE_PURSUIT_H
#define DRIVE_CONTROLLER_PURE_PURSUIT_H

#include <string>

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>

#include "f1tenth_msgs/Trajectory.h"

class PurePursuit
{
private:
  double look_ahead_dist_;
  double gain_;

  std::string robot_frame_id;

  std::shared_ptr<visualization_msgs::MarkerArray> viz_ptr_;

  int findLookAheadWaypointId(const geometry_msgs::Pose& cur_pose, const f1tenth_msgs::Trajectory& trajectory);

public:
  PurePursuit(const double look_ahead_dist, const double gain,
              const std::shared_ptr<visualization_msgs::MarkerArray>& viz_ptr);

  ackermann_msgs::AckermannDriveStamped computeDrive(const geometry_msgs::Pose& cur_pose,
                                                     const f1tenth_msgs::Trajectory& trajectory,
                                                     const std::string& vehicle_frame);

  void setGain(const double gain);

  void setLookAheadDistance(const double distance);

  void visualizeArc(const double curvature, const std::string& robot_frame_id) const;

  void visualizeLookAheadPoint(const geometry_msgs::PointStamped& point) const;
};

#endif  // DRIVE_CONTROLLER_PURE_PURSUIT_H
