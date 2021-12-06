#include <cmath>
#include <string>
#include <vector>

#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "collision_warning_system/bicycle_model.h"

BicycleModel::BicycleModel(const double wheelbase) : wheelbase_(wheelbase)
{
}

nav_msgs::Path BicycleModel::projectTrajectory(const geometry_msgs::Pose initial_pose, const std::string frame_id,
                                               const double velocity, const double steering_angle, const double delta_t,
                                               const double t_max)
{
  tf2::Quaternion yaw_quat;
  tf2::convert(initial_pose.orientation, yaw_quat);
  tf2::Matrix3x3 yaw_mat(yaw_quat);

  double roll, pitch, yaw;
  yaw_mat.getRPY(roll, pitch, yaw);

  BicycleState initial_state(initial_pose.position.x, initial_pose.position.y, velocity, yaw, steering_angle);

  std::vector<BicycleState> projected_trajectory;
  projected_trajectory.push_back(initial_state);
  BicycleState next_state = initial_state;

  for (int i = 0; i < t_max / delta_t; ++i)
  {
    next_state = propagateState(next_state, delta_t);
    projected_trajectory.push_back(next_state);
  }

  return bicycleStatesToPath(projected_trajectory, frame_id);
}

BicycleState BicycleModel::propagateState(const BicycleState state, const double delta_t)
{
  double v_x = state.v() * cos(state.yaw());
  double v_y = state.v() * sin(state.yaw());
  double yaw_rate = state.v() * tan(state.steering_angle()) / wheelbase_;
  double next_x = state.x() + v_x * delta_t;
  double next_y = state.y() + v_y * delta_t;
  double next_yaw = state.yaw() + yaw_rate * delta_t;
  return BicycleState(next_x, next_y, state.v(), next_yaw, state.steering_angle());
}

nav_msgs::Path BicycleModel::bicycleStatesToPath(const std::vector<BicycleState> states, std::string frame_id)
{
  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = frame_id;

  for (auto& state : states)
  {
    path.poses.push_back(bicycleStateToPoseStamped(state, frame_id));
  }

  return path;
}

geometry_msgs::PoseStamped BicycleModel::bicycleStateToPoseStamped(const BicycleState state, std::string frame_id)
{
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = frame_id;
  pose_stamped.header.stamp = ros::Time::now();

  tf2::Quaternion yaw_quat;
  yaw_quat.setRPY(0, 0, state.yaw());
  geometry_msgs::Quaternion yaw_quat_msg;
  tf2::convert(yaw_quat, yaw_quat_msg);

  pose_stamped.pose.position.x = state.x();
  pose_stamped.pose.position.y = state.y();
  pose_stamped.pose.orientation = yaw_quat_msg;

  return pose_stamped;
}
