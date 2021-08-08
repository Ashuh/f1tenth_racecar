#include <algorithm>
#include <string>
#include <vector>

#include <geometry_msgs/Pose2D.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>

#include "costmap_generator/collision_checker.h"
#include "local_planner/path.h"
#include "local_planner/trajectory.h"
#include "local_planner/cubic_spiral.h"
#include "local_planner/tracking_trajectory_generator.h"

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
    const std::shared_ptr<visualization_msgs::MarkerArray>& viz_ptr)
  : tf_listener_(tf_buffer_), sampling_pattern_(sampling_pattern), cubic_spiral_opt_(max_curvature)
{
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
                                                                   const double initial_curvature)
{
  if (reference_trajectory.size() == 0)
  {
    throw std::runtime_error("Reference trajectory is empty");
  }

  int reference_goal_id = getReferenceGoalId(reference_trajectory);
  geometry_msgs::Pose reference_goal = reference_trajectory.pose(reference_goal_id);

  // transform reference_goal to local frame
  geometry_msgs::TransformStamped transform =
      tf_buffer_.lookupTransform("base_link", reference_trajectory.getFrameId(), ros::Time(0));
  tf2::doTransform(reference_goal, reference_goal, transform);

  std::vector<Path> candidate_paths = generateCandidatePaths(reference_goal, initial_curvature);
  std::vector<Path> safe_paths;
  std::vector<Path> unsafe_paths;

  for (auto& path : candidate_paths)
  {
    (checkCollision(path) ? unsafe_paths : safe_paths).push_back(path);
  }

  visualizePaths(safe_paths, unsafe_paths);

  std::vector<Trajectory> trajectories;

  for (auto& path : safe_paths)
  {
    std::vector<double> velocity_profile = generateVelocityProfile(path);
    Trajectory trajectory(path, velocity_profile);
    trajectories.push_back(trajectory);
  }

  std::vector<double> costs;

  for (auto& trajectory : trajectories)
  {
    double cost = evaluateTrajectory(reference_trajectory, trajectory, reference_goal);
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
                                                                      const double initial_curvature)
{
  std::vector<Path> paths;

  for (int i = 0; i < sampling_pattern_.num_paths_; ++i)
  {
    double goal_offset = (i - sampling_pattern_.num_paths_ / 2) * sampling_pattern_.lateral_spacing_;

    geometry_msgs::Pose2D goal = offsetGoal(reference_goal, goal_offset);
    Path path = cubic_spiral_opt_.optimizeCubicSpiral(initial_curvature, 0.0, goal.x, goal.y, goal.theta).toPath(10);
    paths.push_back(path);
  }

  return paths;
}

int TrackingTrajectoryGenerator::getReferenceGoalId(const Trajectory& reference_trajectory)
{
  double time = 0.0;
  int goal_id = 1;

  while (goal_id < reference_trajectory.size() && time < sampling_pattern_.look_ahead_time_)
  {
    time += getArrivalTime(reference_trajectory.distance(goal_id) - reference_trajectory.distance(goal_id - 1),
                           reference_trajectory.velocity(goal_id - 1), reference_trajectory.velocity(goal_id));
    ++goal_id;
  }

  return goal_id - 1;
}

geometry_msgs::Pose2D TrackingTrajectoryGenerator::poseToPose2D(const geometry_msgs::Pose& pose)
{
  tf2::Quaternion quat_tf;
  tf2::fromMsg(pose.orientation, quat_tf);
  double dummy, yaw;
  tf2::Matrix3x3(quat_tf).getEulerYPR(yaw, dummy, dummy);

  geometry_msgs::Pose2D pose_2d;
  pose_2d.x = pose.position.x;
  pose_2d.y = pose.position.y;
  pose_2d.theta = yaw;

  return pose_2d;
}

geometry_msgs::Pose2D TrackingTrajectoryGenerator::offsetGoal(const geometry_msgs::Pose& reference_goal,
                                                              const double lateral_offset)
{
  tf2::Quaternion quat_tf;
  tf2::fromMsg(reference_goal.orientation, quat_tf);
  double dummy, reference_yaw;
  tf2::Matrix3x3(quat_tf).getEulerYPR(reference_yaw, dummy, dummy);

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

double TrackingTrajectoryGenerator::getArrivalTime(const double s, const double v_i, const double v_f)
{
  return 2 * s / (v_i + v_f);
}

std::vector<double> TrackingTrajectoryGenerator::generateVelocityProfile(const Path& path)
{
  // placeholder

  std::vector<double> velocity_profile;

  for (int i = 0; i < path.size(); ++i)
  {
    velocity_profile.push_back(1);
  }

  return velocity_profile;
}

double TrackingTrajectoryGenerator::evaluateTrajectory(const Trajectory& reference_trajectory,
                                                       const Trajectory& trajectory, const geometry_msgs::Pose& goal)
{
  // placeholder
  double d_x = trajectory.x(trajectory.size() - 1) - goal.position.x;
  double d_y = trajectory.y(trajectory.size() - 1) - goal.position.y;

  return pow(d_x, 2) + pow(d_y, 2);
}

void TrackingTrajectoryGenerator::setCostmap(const grid_map_msgs::GridMap::ConstPtr& costmap_msg)
{
  grid_map::GridMapRosConverter::fromMessage(*costmap_msg, costmap_);
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
