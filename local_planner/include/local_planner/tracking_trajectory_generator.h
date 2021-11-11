#ifndef LOCAL_PLANNER_TRACKING_TRAJECTORY_GENERATOR_H
#define LOCAL_PLANNER_TRACKING_TRAJECTORY_GENERATOR_H

#include <string>
#include <vector>

#include <geometry_msgs/Pose2D.h>
#include <grid_map_msgs/GridMap.h>
#include <visualization_msgs/MarkerArray.h>

#include "costmap_generator/collision_checker.h"
#include "local_planner/cubic_spiral.h"
#include "local_planner/path.h"
#include "local_planner/trajectory.h"
#include "local_planner/trajectory_evaluator.h"

class TrackingTrajectoryGenerator
{
public:
  TrackingTrajectoryGenerator(const int num_paths, const double lateral_spacing, const double look_ahead_time,
                              const double max_curvature,
                              const std::shared_ptr<CollisionChecker>& collision_checker_ptr,
                              const std::shared_ptr<TrajectoryEvaluator>& trajectory_evaluator_ptr,
                              const std::shared_ptr<visualization_msgs::MarkerArray>& viz_ptr);

  Trajectory generateTrackingTrajectory(const Trajectory& reference_trajectory, const double initial_velocity,
                                        const double initial_curvature);

  void setNumPaths(const int num_paths);

  void setLateralSpacing(const double spacing);

  void setLookAheadTime(const double time);

private:
  int num_paths_;
  double lateral_spacing_;
  double look_ahead_time_;

  CubicSpiral::OptimizerNLOPT cubic_spiral_opt_;

  std::shared_ptr<CollisionChecker> collision_checker_ptr_;
  std::shared_ptr<TrajectoryEvaluator> trajectory_evaluator_ptr_;
  std::shared_ptr<visualization_msgs::MarkerArray> viz_ptr_;

  std::vector<Path> generateCandidatePaths(const geometry_msgs::Pose& reference_goal, const int num_wp,
                                           const double initial_curvature);

  geometry_msgs::Pose2D poseToPose2D(const geometry_msgs::Pose& pose);

  geometry_msgs::Pose2D offsetGoal(const geometry_msgs::Pose& reference_goal, const double lateral_offset);

  bool checkCollision(const Path& trajectory);

  void visualizePaths(const std::vector<Path>& safe_paths, const std::vector<Path>& unsafe_paths);

  void visualizeFinalTrajectory(const Trajectory& trajectory);
};

#endif  // LOCAL_PLANNER_TRACKING_TRAJECTORY_GENERATOR_H
