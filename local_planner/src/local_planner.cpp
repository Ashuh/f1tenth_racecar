#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <grid_map_msgs/GridMap.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include "local_planner/path.h"
#include "local_planner/trajectory.h"
#include "local_planner/cubic_spiral_optimizer.h"
#include "local_planner/velocity_profile_generator.h"
#include "local_planner/trajectory_evaluator.h"
#include "local_planner/local_planner.h"

LocalPlanner::LocalPlanner() : tf_listener_(tf_buffer_)
{
  ros::NodeHandle private_nh("~");

  std::string costmap_topic = "costmap";
  std::string global_path_topic = "path/global";
  std::string local_path_topic = "path/local";
  std::string odom_topic = "odom";
  std::string viz_topic = "viz/local_planner";

  double max_lat_acc = 1;
  double max_long_acc = 1;
  double wheelbase = 0.3;
  double max_steering_angle = 0.4;

  // ROS_ASSERT(private_nh.getParam("global_path_topic", global_path_topic));
  // ROS_ASSERT(private_nh.getParam("local_path_topic", local_path_topic));
  // ROS_ASSERT(private_nh.getParam("odom_topic", odom_topic));
  // ROS_ASSERT(private_nh.getParam("viz_topic", viz_topic));

  // ROS_ASSERT(private_nh.getParam("max_lat_acc", max_lat_acc));
  // ROS_ASSERT(private_nh.getParam("max_long_acc", max_long_acc));
  // ROS_ASSERT(private_nh.getParam("wheelbase", wheelbase));
  // ROS_ASSERT(private_nh.getParam("max_steering_angle", max_steering_angle));
  // ROS_ASSERT(private_nh.getParam("num_paths", num_paths_));
  // ROS_ASSERT(private_nh.getParam("path_offset", path_offset_));
  // ROS_ASSERT(private_nh.getParam("look_ahead_time", look_ahead_time_));
  // ROS_ASSERT(private_nh.getParam("min_look_ahead_dist", min_look_ahead_dist_));

  global_path_sub_ = nh_.subscribe(global_path_topic, 1, &LocalPlanner::globalPathCallback, this);
  odom_sub_ = nh_.subscribe(odom_topic, 1, &LocalPlanner::odomCallback, this);
  costmap_sub_ = nh_.subscribe(costmap_topic, 1, &LocalPlanner::costmapCallback, this);

  local_path_pub_ = nh_.advertise<nav_msgs::Path>(local_path_topic, 1);
  viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(viz_topic, 1);

  timer_ = nh_.createTimer(ros::Duration(0.1), &LocalPlanner::timerCallback, this);

  num_paths_ = 10;
  path_offset_ = 0.2;
  look_ahead_time_ = 2.0;
  min_look_ahead_dist_ = 2.0;

  opt_ = std::make_unique<CubicSpiralOptimizer>(tan(max_steering_angle) / wheelbase);
  velocity_gen_ = std::make_unique<VelocityProfileGenerator>(max_lat_acc, max_long_acc);
  traj_eval_ = std::make_unique<TrajectoryEvaluator>();
}

void LocalPlanner::timerCallback(const ros::TimerEvent& timer_event)
{
  geometry_msgs::Pose2D reference_goal;

  try
  {
    reference_goal = getReferenceGoal();
  }
  catch (const std::exception& ex)
  {
    ROS_ERROR("[Local Planner] %s", ex.what());
    return;
  }

  std::vector<Trajectory> trajectories;
  std::vector<double> costs;

  for (int i = 0; i < num_paths_; ++i)
  {
    double goal_offset = (i - num_paths_ / 2) * path_offset_;

    geometry_msgs::Pose2D goal = generateOffsetGoal(reference_goal, goal_offset);
    Path path = opt_->generateCubicSpiralPath(goal.x, goal.y, goal.theta, 10);
    Trajectory trajectory = velocity_gen_->generateVelocityProfile(path, latest_odom_.twist.twist.linear.x, 5);
    double traj_cost = traj_eval_->evaluateTrajectory(trajectory, goal_offset);

    trajectories.push_back(trajectory);
    costs.push_back(traj_cost);
  }

  int best_traj_id = std::min_element(costs.begin(), costs.end()) - costs.begin();

  if (costs.at(best_traj_id) != std::numeric_limits<double>::max())
  {
    nav_msgs::Path local_path_msg =
        trajectoryToPathMsg(trajectories.at(best_traj_id));  // use standard path msg for now
    local_path_pub_.publish(local_path_msg);
  }

  visualizePaths(trajectories, costs);
}

void LocalPlanner::globalPathCallback(const nav_msgs::Path& path_msg)
{
  global_path_ = path_msg;
}

void LocalPlanner::odomCallback(const nav_msgs::Odometry& odom_msg)
{
  latest_odom_ = odom_msg;
}

void LocalPlanner::costmapCallback(const grid_map_msgs::GridMap::ConstPtr& costmap_msg)
{
  traj_eval_->setCostmap(costmap_msg);
}

void LocalPlanner::visualizePaths(const std::vector<Trajectory>& trajectories, const std::vector<double>& costs)
{
  visualization_msgs::MarkerArray trajectory_marker_arr;

  for (int i = 0; i < trajectories.size(); ++i)
  {
    Trajectory trajectory = trajectories.at(i);

    visualization_msgs::Marker trajectory_marker;
    trajectory_marker.header.frame_id = "base_link";
    trajectory_marker.action = visualization_msgs::Marker::ADD;
    trajectory_marker.type = visualization_msgs::Marker::LINE_STRIP;
    trajectory_marker.ns = "waypoints";
    trajectory_marker.id = i;
    trajectory_marker.lifetime = ros::Duration(0.1);
    trajectory_marker.scale.x = 0.02;
    trajectory_marker.color.r = 0;
    trajectory_marker.color.g = 1;
    trajectory_marker.color.b = 0;
    trajectory_marker.color.a = 1;

    if (costs.at(i) == std::numeric_limits<double>::max())
    {
      trajectory_marker.color.r = 1;
    }

    for (int j = 0; j < trajectory.size(); ++j)
    {
      geometry_msgs::Point point;
      point.x = trajectory.x(j);
      point.y = trajectory.y(j);
      trajectory_marker.points.push_back(point);

      visualization_msgs::Marker velocity_marker;
      velocity_marker.header.frame_id = "base_link";
      velocity_marker.action = visualization_msgs::Marker::ADD;
      velocity_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      velocity_marker.ns = "velocity_profile";
      velocity_marker.id = i * trajectory.size() + j;
      velocity_marker.lifetime = ros::Duration(0.1);
      velocity_marker.scale.z = 0.1;
      velocity_marker.color.r = 1;
      velocity_marker.color.g = 1;
      velocity_marker.color.b = 1;
      velocity_marker.color.a = 1;

      std::ostringstream oss;
      oss << std::fixed << std::setprecision(2) << 0;  // temp value
      velocity_marker.text = oss.str();

      velocity_marker.pose.position = point;
      velocity_marker.pose.position.z = 0.1;
      trajectory_marker_arr.markers.push_back(velocity_marker);
    }

    trajectory_marker_arr.markers.push_back(trajectory_marker);
  }

  viz_pub_.publish(trajectory_marker_arr);
}

int LocalPlanner::getNearestWaypointId()
{
  int nearest_wp_id = -1;
  double nearest_wp_dist = std::numeric_limits<double>::max();

  for (int i = 0; i < global_path_.poses.size(); ++i)
  {
    double d_x = latest_odom_.pose.pose.position.x - global_path_.poses.at(i).pose.position.x;
    double d_y = latest_odom_.pose.pose.position.y - global_path_.poses.at(i).pose.position.y;
    double dist = sqrt(pow(d_x, 2) + pow(d_y, 2));

    if (dist < nearest_wp_dist)
    {
      nearest_wp_id = i;
      nearest_wp_dist = dist;
    }
  }

  return nearest_wp_id;
}

geometry_msgs::Pose2D LocalPlanner::getReferenceGoal()
{
  if (global_path_.poses.empty())
  {
    throw std::runtime_error("Global Path is empty");
  }

  int nearest_wp_id = getNearestWaypointId();

  double path_distance = 0.0;
  double look_ahead_dist = std::max(look_ahead_time_ * latest_odom_.twist.twist.linear.x, min_look_ahead_dist_);
  int reference_goal_id = (nearest_wp_id == 0) ? 1 : nearest_wp_id;

  while (reference_goal_id < global_path_.poses.size() && path_distance < look_ahead_dist)
  {
    double d_x = global_path_.poses.at(reference_goal_id).pose.position.x -
                 global_path_.poses.at(reference_goal_id - 1).pose.position.x;
    double d_y = global_path_.poses.at(reference_goal_id).pose.position.y -
                 global_path_.poses.at(reference_goal_id - 1).pose.position.y;
    double dist = sqrt(pow(d_x, 2) + pow(d_y, 2));
    path_distance += dist;
    ++reference_goal_id;
  }

  geometry_msgs::TransformStamped transform =
      tf_buffer_.lookupTransform(latest_odom_.child_frame_id, global_path_.header.frame_id, ros::Time(0));
  geometry_msgs::PoseStamped reference_goal_transformed;
  tf2::doTransform(global_path_.poses.at(reference_goal_id), reference_goal_transformed, transform);

  tf2::Quaternion quat_tf;
  tf2::fromMsg(reference_goal_transformed.pose.orientation, quat_tf);
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat_tf).getEulerYPR(yaw, pitch, roll);

  geometry_msgs::Pose2D reference_goal_2d;
  reference_goal_2d.x = reference_goal_transformed.pose.position.x;
  reference_goal_2d.y = reference_goal_transformed.pose.position.y;
  reference_goal_2d.theta = yaw;

  return reference_goal_2d;
}

geometry_msgs::Pose2D LocalPlanner::generateOffsetGoal(const geometry_msgs::Pose2D& reference_goal,
                                                       const double lateral_offset)
{
  double x_offset = lateral_offset * cos(reference_goal.theta + M_PI_2);
  double y_offset = lateral_offset * sin(reference_goal.theta + M_PI_2);

  geometry_msgs::Pose2D goal;
  goal.x = reference_goal.x + x_offset;
  goal.y = reference_goal.y + y_offset;
  goal.theta = reference_goal.theta;

  return goal;
}

// std::vector<geometry_msgs::Pose2D> LocalPlanner::generateGoals(const geometry_msgs::Pose2D& reference_goal,
//                                                                const int num_goals, const double lateral_spacing)
// {
//   std::vector<geometry_msgs::Pose2D> goals;

//   for (int i = 0; i < num_goals; ++i)
//   {
//     double goal_offset = (i - num_goals / 2) * lateral_spacing;
//     double x_offset = goal_offset * cos(reference_goal.theta + M_PI_2);
//     double y_offset = goal_offset * sin(reference_goal.theta + M_PI_2);

//     geometry_msgs::Pose2D goal;
//     goal.x = reference_goal.x + x_offset;
//     goal.y = reference_goal.y + y_offset;
//     goal.theta = reference_goal.theta;

//     goals.push_back(goal);
//   }

//   return goals;
// }

nav_msgs::Path LocalPlanner::trajectoryToPathMsg(const Trajectory& trajectory)
{
  geometry_msgs::TransformStamped transform;

  try
  {
    transform = tf_buffer_.lookupTransform("map", trajectory.getFrameId(), ros::Time(0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
  }

  geometry_msgs::PoseStamped reference_goal_transformed;
  nav_msgs::Path path_msg;

  path_msg.header.frame_id = "map";

  for (int i = 0; i < trajectory.size(); ++i)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = trajectory.getFrameId();
    pose.pose.position.x = trajectory.x(i);
    pose.pose.position.y = trajectory.y(i);

    tf2::Quaternion yaw_quat;
    yaw_quat.setRPY(0, 0, trajectory.yaw(i));
    tf2::convert(yaw_quat, pose.pose.orientation);

    tf2::doTransform(pose, pose, transform);

    path_msg.poses.push_back(pose);
  }

  return path_msg;
}
