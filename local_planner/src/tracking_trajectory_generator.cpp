#include <algorithm>
#include <string>
#include <vector>

#include <geometry_msgs/Pose2D.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <visualization_msgs/MarkerArray.h>

#include "costmap_generator/collision_checker.h"
#include "f1tenth_utils/tf2_wrapper.h"
#include "local_planner/cubic_spiral.h"
#include "local_planner/cubic_velocity_time_profile.h"
#include "local_planner/path.h"
#include "local_planner/tracking_trajectory_generator.h"
#include "local_planner/trajectory.h"
#include "local_planner/trajectory_evaluator.h"

TrackingTrajectoryGenerator::SamplingPattern::SamplingPattern(const int num_paths, const double lateral_spacing,
                                                              const double look_ahead_time)
{
  if (num_paths < 1 || lateral_spacing < 0.0 || look_ahead_time < 0.0)
  {
    throw std::invalid_argument("Invalid sampling pattern specified");
  }

  num_paths_ = num_paths;
  lateral_spacing_ = lateral_spacing;
  look_ahead_time_ = look_ahead_time;
}

TrackingTrajectoryGenerator::TrackingTrajectoryGenerator(
    const SamplingPattern& sampling_pattern, const double max_curvature,
    const std::shared_ptr<CollisionChecker>& collision_checker_ptr,
    const std::shared_ptr<TrajectoryEvaluator>& trajectory_evaluator_ptr,
    const std::shared_ptr<visualization_msgs::MarkerArray>& viz_ptr)
  : sampling_pattern_(sampling_pattern), cubic_spiral_opt_(max_curvature)
{
  trajectory_evaluator_ptr_ = trajectory_evaluator_ptr;
  viz_ptr_ = viz_ptr;

  if (collision_checker_ptr != nullptr)
  {
    collision_checker_ptr_ = collision_checker_ptr;
  }
  else
  {
    throw std::invalid_argument("Collision checker pointer is null");
  }
}

Trajectory TrackingTrajectoryGenerator::generateTrackingTrajectory(const Trajectory& reference_trajectory,
                                                                   const double initial_velocity,
                                                                   const double initial_curvature)
{
  if (reference_trajectory.size() == 0)
  {
    throw std::runtime_error("Reference trajectory is empty");
  }

  Trajectory ref_traj_trimmed =
      reference_trajectory.trim(0, reference_trajectory.getWpIdAtTime(sampling_pattern_.look_ahead_time_));

  geometry_msgs::Pose reference_goal = ref_traj_trimmed.pose(ref_traj_trimmed.size() - 1);

  std::vector<Path> candidate_paths =
      generateCandidatePaths(reference_goal, ref_traj_trimmed.size(), initial_curvature);
  auto partition = std::partition(candidate_paths.begin(), candidate_paths.end(),
                                  [this](const Path& p) { return checkCollision(p); });
  std::vector<Path> unsafe_paths(candidate_paths.begin(), partition);
  std::vector<Path> safe_paths(partition, candidate_paths.end());

  visualizePaths(safe_paths, unsafe_paths);

  std::vector<Trajectory> trajectories;

  for (auto& path : safe_paths)
  {
    CubicVelocityTimeProfile profile(initial_velocity, ref_traj_trimmed.velocity(ref_traj_trimmed.size() - 1),
                                     path.distance(path.size() - 1));

    trajectories.push_back(Trajectory(path, profile));
  }

  trajectory_evaluator_ptr_->setReferenceTrajectory(ref_traj_trimmed);
  std::vector<double> costs;

  for (auto& trajectory : trajectories)
  {
    double cost = trajectory_evaluator_ptr_->evaluate(trajectory);
    costs.push_back(cost);
  }

  if (trajectories.empty())
  {
    throw std::runtime_error("No safe paths found");
  }
  else
  {
    int best_traj_id = std::min_element(costs.begin(), costs.end()) - costs.begin();
    return trajectories.at(best_traj_id);
  }
}

std::vector<Path> TrackingTrajectoryGenerator::generateCandidatePaths(const geometry_msgs::Pose& reference_goal,
                                                                      const int num_wp, const double initial_curvature)
{
  std::vector<Path> paths;

  for (int i = 0; i < sampling_pattern_.num_paths_; ++i)
  {
    double goal_offset = (i - sampling_pattern_.num_paths_ / 2) * sampling_pattern_.lateral_spacing_;

    geometry_msgs::Pose2D goal = offsetGoal(reference_goal, goal_offset);
    Path path =
        cubic_spiral_opt_.optimizeCubicSpiral(initial_curvature, 0.0, goal.x, goal.y, goal.theta).toPath(num_wp);
    paths.push_back(path);
  }

  return paths;
}

geometry_msgs::Pose2D TrackingTrajectoryGenerator::poseToPose2D(const geometry_msgs::Pose& pose)
{
  geometry_msgs::Pose2D pose_2d;
  pose_2d.x = pose.position.x;
  pose_2d.y = pose.position.y;
  pose_2d.theta = TF2Wrapper::yawFromQuat(pose.orientation);

  return pose_2d;
}

geometry_msgs::Pose2D TrackingTrajectoryGenerator::offsetGoal(const geometry_msgs::Pose& reference_goal,
                                                              const double lateral_offset)
{
  double reference_yaw = TF2Wrapper::yawFromQuat(reference_goal.orientation);

  double x_offset = lateral_offset * cos(reference_yaw + M_PI_2);
  double y_offset = lateral_offset * sin(reference_yaw + M_PI_2);

  geometry_msgs::Pose2D goal;
  goal.x = reference_goal.position.x + x_offset;
  goal.y = reference_goal.position.y + y_offset;
  goal.theta = reference_yaw;

  return goal;
}

bool TrackingTrajectoryGenerator::checkCollision(const Path& path)
{
  try
  {
    for (int i = 0; i < path.size(); ++i)
    {
      if (collision_checker_ptr_->checkCollision(path.poseStamped(i)))
      {
        return true;
      }
    }
  }
  catch (const std::exception& ex)
  {
    ROS_ERROR("[Local Planner] TTG Collision check failed due to %s. Assuming there is a collision.", ex.what());
    return true;
  }

  return false;
}

void TrackingTrajectoryGenerator::setCostmap(const grid_map_msgs::GridMap::ConstPtr& costmap_msg)
{
  collision_checker_ptr_->setCostmap(costmap_msg);
}

void TrackingTrajectoryGenerator::setSamplingPattern(const SamplingPattern& pattern)
{
  sampling_pattern_ = pattern;
}

void TrackingTrajectoryGenerator::visualizePaths(const std::vector<Path>& safe_paths,
                                                 const std::vector<Path>& unsafe_paths)
{
  visualization_msgs::MarkerArray path_markers;

  for (int i = 0; i < safe_paths.size(); ++i)
  {
    path_markers.markers.push_back(safe_paths.at(i).generatePathMarker(i, "safe_paths", 0.02, 0, 1, 0));
  }

  for (int i = 0; i < unsafe_paths.size(); ++i)
  {
    path_markers.markers.push_back(unsafe_paths.at(i).generatePathMarker(i, "unsafe_paths", 0.02, 1, 0, 0));
  }

  viz_ptr_->markers.insert(viz_ptr_->markers.end(), path_markers.markers.begin(), path_markers.markers.end());
}
