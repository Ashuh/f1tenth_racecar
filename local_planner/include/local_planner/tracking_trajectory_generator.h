#ifndef LOCAL_PLANNER_TRACKING_TRAJECTORY_GENERATOR_H
#define LOCAL_PLANNER_TRACKING_TRAJECTORY_GENERATOR_H

#include <vector>

#include <geometry_msgs/Pose2D.h>
#include <grid_map_msgs/GridMap.h>
#include <tf2_ros/transform_listener.h>

#include "local_planner/path.h"
#include "local_planner/trajectory.h"
#include "local_planner/cubic_spiral_optimizer.h"

class TrackingTrajectoryGenerator
{
private:
  int num_paths_;
  double lateral_spacing_;
  double look_ahead_time_;

  grid_map::GridMap costmap_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::unique_ptr<CubicSpiralOptimizer> cubic_spiral_opt_;

  std::vector<Path> generateCandidatePaths(const geometry_msgs::Pose& reference_goal, const double initial_curvature);

  int getReferenceGoalId(const Trajectory& reference_trajectory);

  geometry_msgs::Pose2D poseToPose2D(const geometry_msgs::Pose& pose);

  double getArrivalTime(const double s, const double v_i, const double v_f);

  geometry_msgs::Pose2D offsetGoal(const geometry_msgs::Pose& reference_goal, const double lateral_offset);

  bool checkCollision(const Path& trajectory);

  std::vector<double> generateVelocityProfile(const Path& path);

  double evaluateTrajectory(const Trajectory& reference_trajectory, const Trajectory& trajectory,
                            const geometry_msgs::Pose& goal);

public:
  TrackingTrajectoryGenerator(const int num_paths, const double max_curvature, const double lateral_spacing,
                              const double look_ahead_time);

  Trajectory generateTrackingTrajectory(const Trajectory& reference_trajectory, const double initial_curvature,
                                        std::vector<Path>& safe_paths, std::vector<Path>& unsafe_paths);

  void setCostmap(const grid_map_msgs::GridMap::ConstPtr& costmap_msg);
};

#endif  // LOCAL_PLANNER_TRACKING_TRAJECTORY_GENERATOR_H
