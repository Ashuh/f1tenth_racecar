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
  struct SamplingPattern
  {
    int num_paths_;
    double lateral_spacing_;
    double look_ahead_time_;

    SamplingPattern(const int num_paths, const double lateral_spacing, const double look_ahead_time);
  };

  TrackingTrajectoryGenerator(const SamplingPattern& sampling_pattern, const double max_curvature,
                              const std::shared_ptr<CollisionChecker>& collision_checker_ptr,
                              const std::shared_ptr<TrajectoryEvaluator>& trajectory_evaluator_ptr,
                              const std::shared_ptr<visualization_msgs::MarkerArray>& viz_ptr = nullptr);

  Trajectory generateTrackingTrajectory(const Trajectory& reference_trajectory, const double initial_velocity,
                                        const double initial_curvature);

  void setCostmap(const grid_map_msgs::GridMap::ConstPtr& costmap_msg);

  void setSamplingPattern(const SamplingPattern& pattern);

private:
  SamplingPattern sampling_pattern_;

  // CubicSpiral::OptimizerIFOPT cubic_spiral_opt_;
  CubicSpiral::OptimizerNLOPT cubic_spiral_opt_;

  std::shared_ptr<CollisionChecker> collision_checker_ptr_;
  std::shared_ptr<TrajectoryEvaluator> trajectory_evaluator_ptr_;
  std::shared_ptr<visualization_msgs::MarkerArray> viz_ptr_;

  std::vector<Path> generateCandidatePaths(const geometry_msgs::Pose& reference_goal, const int num_wp,
                                           const double initial_curvature);

  geometry_msgs::Pose2D poseToPose2D(const geometry_msgs::Pose& pose);

  geometry_msgs::Pose2D offsetGoal(const geometry_msgs::Pose& reference_goal, const double lateral_offset);

  bool checkCollision(const Path& trajectory);

  double evaluateTrajectory(const Trajectory& reference_trajectory, const Trajectory& trajectory,
                            const geometry_msgs::Pose& goal);

  void visualizePaths(const std::vector<Path>& safe_paths, const std::vector<Path>& unsafe_paths);
};

#endif  // LOCAL_PLANNER_TRACKING_TRAJECTORY_GENERATOR_H
