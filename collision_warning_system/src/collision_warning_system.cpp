#include <string>
#include <vector>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "collision_warning_system/collision_warning_system.h"

namespace f1tenth_racecar
{
namespace safety
{
CollisionWarningSystem::CollisionWarningSystem() : tf_listener(tf_buffer)
{
  ros::NodeHandle private_nh("~");
  timer_ = nh_.createTimer(ros::Duration(0.1), &CollisionWarningSystem::timerCallback, this);

  std::string odom_topic = "odom";
  std::string drive_topic = "drive";
  std::string trajectory_topic = "trajectory";
  double wheel_base = 0.3;
  model_ = BicycleModel(wheel_base);

  odom_sub_ = nh_.subscribe(odom_topic, 1, &CollisionWarningSystem::odomCallback, this);
  drive_sub_ = nh_.subscribe(drive_topic, 1, &CollisionWarningSystem::driveCallback, this);
  trajectory_pub_ = nh_.advertise<nav_msgs::Path>(trajectory_topic, 1);
}

void CollisionWarningSystem::timerCallback(const ros::TimerEvent& timer_event)
{
  geometry_msgs::TransformStamped map_to_base_link;

  try
  {
    map_to_base_link = tf_buffer.lookupTransform("base_link", "map", ros::Time(0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }

  geometry_msgs::Pose transformed_pose;
  tf2::doTransform(odom_msg_.pose.pose, transformed_pose, map_to_base_link);

  tf2::Quaternion yaw_quat;
  tf2::convert(transformed_pose.orientation, yaw_quat);
  tf2::Matrix3x3 yaw_mat(yaw_quat);

  double roll;
  double pitch;
  double yaw;

  yaw_mat.getRPY(roll, pitch, yaw);

  BicycleState initial_state(transformed_pose.position.x, transformed_pose.position.y, odom_msg_.twist.twist.linear.x,
                             yaw, drive_msg_.drive.steering_angle);
  nav_msgs::Path projected_trajectory = bicycleStatesToPath(model_.projectTrajectory(initial_state, delta_t_, steps_));
  trajectory_pub_.publish(projected_trajectory);
}

void CollisionWarningSystem::odomCallback(const nav_msgs::Odometry odom_msg)
{
  odom_msg_ = odom_msg;
}

void CollisionWarningSystem::driveCallback(const ackermann_msgs::AckermannDriveStamped drive_msg)
{
  drive_msg_ = drive_msg;
}

nav_msgs::Path CollisionWarningSystem::bicycleStatesToPath(const std::vector<BicycleState> states)
{
  nav_msgs::Path path;
  path.header.frame_id = "base_link";
  path.header.stamp = ros::Time::now();

  for (auto& state : states)
  {
    path.poses.push_back(bicycleStateToPoseStamped(state));
  }

  return path;
}

geometry_msgs::PoseStamped CollisionWarningSystem::bicycleStateToPoseStamped(const BicycleState state)
{
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = "base_link";
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
}  // namespace safety
}  // namespace f1tenth_racecar

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "collision_warning_system");
  f1tenth_racecar::safety::CollisionWarningSystem cws;
  ros::spin();
  return 0;
}
