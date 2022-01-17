#include <algorithm>
#include <string>
#include <vector>

#include <geometry_msgs/Pose2D.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <visualization_msgs/MarkerArray.h>

#include "f1tenth_utils/tf2_wrapper.h"
#include "local_planner/cubic_spiral.h"
#include "local_planner/cubic_velocity_time_profile.h"
#include "local_planner/path.h"
#include "local_planner/tracking_trajectory_generator.h"
#include "local_planner/trajectory.h"
#include "local_planner/trajectory_evaluator.h"

TrackingTrajectoryGenerator::TrackingTrajectoryGenerator(
    const int num_paths, const double lateral_spacing, const double look_ahead_time, const double max_curvature,
    const std::shared_ptr<TrajectoryEvaluator>& trajectory_evaluator_ptr,
    const std::shared_ptr<visualization_msgs::MarkerArray>& viz_ptr)
  : cubic_spiral_opt_(max_curvature)
{
  setNumPaths(num_paths);
  setLateralSpacing(lateral_spacing);
  setLookAheadTime(look_ahead_time);

  if (trajectory_evaluator_ptr == nullptr)
  {
    throw std::invalid_argument("Trajectory evaluator cannot be nullptr");
  }

  trajectory_evaluator_ptr_ = trajectory_evaluator_ptr;
  viz_ptr_ = viz_ptr;
}

Trajectory TrackingTrajectoryGenerator::generateTrackingTrajectory(const Trajectory& reference_trajectory,
                                                                   const double initial_velocity,
                                                                   const double initial_curvature)
{
  if (reference_trajectory.size() == 0)
  {
    throw std::runtime_error("Reference trajectory is empty");
  }

  Trajectory ref_traj_trimmed = reference_trajectory.trim(0, reference_trajectory.getWpIdAtTime(look_ahead_time_));

  geometry_msgs::Pose reference_goal = ref_traj_trimmed.pose(ref_traj_trimmed.size() - 1);

  std::vector<Path> candidate_paths =
      generateCandidatePaths(reference_goal, ref_traj_trimmed.size(), initial_curvature);

  std::vector<Trajectory> trajectories;

  for (auto& path : candidate_paths)
  {
    CubicVelocityTimeProfile profile(initial_velocity, ref_traj_trimmed.velocity(ref_traj_trimmed.size() - 1),
                                     path.distance(path.size() - 1));

    trajectories.push_back(Trajectory(path, profile));
  }

  trajectory_evaluator_ptr_->setReferenceTrajectory(ref_traj_trimmed);
  std::vector<double> costs;
  std::vector<Trajectory> safe_trajectories;
  std::vector<Path> unsafe_paths;
  std::vector<Path> safe_paths;

  for (auto& trajectory : trajectories)
  {
    double cost = trajectory_evaluator_ptr_->evaluate(trajectory);

    if (cost < std::numeric_limits<double>::max())
    {
      safe_trajectories.push_back(trajectory);
      safe_paths.push_back(trajectory);
      costs.push_back(cost);
    }
    else
    {
      unsafe_paths.push_back(trajectory);
    }
  }

  visualizePaths(safe_paths, unsafe_paths);

  if (safe_trajectories.empty())
  {
    throw std::runtime_error("No safe paths found");
  }

  int best_traj_id = std::min_element(costs.begin(), costs.end()) - costs.begin();

  Trajectory final_trajectory = safe_trajectories.at(best_traj_id);
  visualizeFinalTrajectory(final_trajectory);
  return final_trajectory;
}

void TrackingTrajectoryGenerator::setNumPaths(const int num_paths)
{
  if (num_paths < 1)
  {
    throw std::invalid_argument("Number of paths must be at least 1");
  }

  num_paths_ = num_paths;
}

void TrackingTrajectoryGenerator::setLateralSpacing(const double spacing)
{
  if (spacing <= 0.0)
  {
    throw std::invalid_argument("Lateral spacing must be positive");
  }

  lateral_spacing_ = spacing;
}

void TrackingTrajectoryGenerator::setLookAheadTime(const double time)
{
  if (time <= 0.0)
  {
    throw std::invalid_argument("Look ahead time must be positive");
  }

  look_ahead_time_ = time;
}

std::vector<Path> TrackingTrajectoryGenerator::generateCandidatePaths(const geometry_msgs::Pose& reference_goal,
                                                                      const int num_wp, const double initial_curvature)
{
  std::vector<Path> paths;

  for (int i = 0; i < num_paths_; ++i)
  {
    double goal_offset = (i - num_paths_ / 2) * lateral_spacing_;

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

double TrackingTrajectoryGenerator::checkCollision(const Path& path)
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

void TrackingTrajectoryGenerator::visualizePaths(const std::vector<Path>& safe_paths,
                                                 const std::vector<Path>& unsafe_paths)
{
  if (viz_ptr_ == nullptr)
  {
    return;
  }

  int id = 0;

  for (const auto& path : safe_paths)
  {
    viz_ptr_->markers.push_back(path.generateLineMarker(id++, "safe_paths", 0.02, 0.0, 0.0, 1.0, 0.0, 0.8));
  }

  for (const auto& path : unsafe_paths)
  {
    viz_ptr_->markers.push_back(path.generateLineMarker(id++, "unsafe_paths", 0.02, 0.0, 1.0, 0.0, 0.0, 0.8));
  }
}

void TrackingTrajectoryGenerator::visualizeFinalTrajectory(const Trajectory& trajectory)
{
  if (viz_ptr_ == nullptr)
  {
    return;
  }

  visualization_msgs::MarkerArray velocity_markers =
      trajectory.generateVelocityMarkers(0, "final_trajectory/velocity", 0.05, 0.2, 0.0, 1.0, 0.0);
  visualization_msgs::MarkerArray arrow_markers =
      trajectory.generateArrowMarkers(0, "final_trajectory/path", 0.03, 0.2, 0.05, 0.0, 1.0, 0.0);

  viz_ptr_->markers.insert(viz_ptr_->markers.end(), velocity_markers.markers.begin(), velocity_markers.markers.end());
  viz_ptr_->markers.insert(viz_ptr_->markers.end(), arrow_markers.markers.begin(), arrow_markers.markers.end());
}
