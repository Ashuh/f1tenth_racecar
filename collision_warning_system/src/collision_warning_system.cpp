#include <limits>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>

#include "collision_warning_system/collision_warning_system.h"
#include "f1tenth_utils/tf2_wrapper.h"

namespace f1tenth_racecar
{
namespace safety
{
CollisionWarningSystem::CollisionWarningSystem()
{
  ros::NodeHandle private_nh("~");

  std::vector<double> circle_offsets;
  double circle_radius;
  double wheelbase;

  std::string odom_topic;
  std::string drive_topic;
  std::string time_to_collision_topic;
  std::string collision_viz_topic;

  private_nh.param("circle_offsets", circle_offsets, std::vector<double>{ 0.1, 0.3 });
  private_nh.param("circle_radius", circle_radius, 0.2);
  private_nh.getParam("t_max", t_max_);
  private_nh.getParam("delta_t", delta_t_);

  private_nh.getParam("wheelbase", wheelbase);
  private_nh.getParam("vehicle_width", vehicle_width_);
  private_nh.getParam("vehicle_length", vehicle_length_);
  private_nh.getParam("base_link_to_center_dist", base_link_to_center_dist_);

  private_nh.getParam("odom_topic", odom_topic);
  private_nh.getParam("drive_topic", drive_topic);
  private_nh.getParam("time_to_collision_topic", time_to_collision_topic);
  private_nh.getParam("collision_viz_topic", collision_viz_topic);

  biycle_model_ = new BicycleModel(wheelbase);
  collision_checker_ = std::make_unique<CollisionChecker>(circle_offsets, circle_radius);
  time_to_collision_pub_ = nh_.advertise<std_msgs::Float64>(time_to_collision_topic, 1);
  viz_pub_ = nh_.advertise<visualization_msgs::Marker>(collision_viz_topic, 1);

  odom_sub_ = nh_.subscribe(odom_topic, 1, &CollisionWarningSystem::odomCallback, this);
  drive_sub_ = nh_.subscribe(drive_topic, 1, &CollisionWarningSystem::driveCallback, this);
  timer_ = nh_.createTimer(ros::Duration(0.1), &CollisionWarningSystem::timerCallback, this);
}

void CollisionWarningSystem::timerCallback(const ros::TimerEvent& timer_event)
{
  try
  {
    geometry_msgs::PoseStamped current_pose;
    current_pose.header = odom_msg_.header;
    current_pose.pose = odom_msg_.pose.pose;
    current_pose = TF2Wrapper::doTransform(current_pose, map_frame_);

    nav_msgs::Path projected_trajectory =
        biycle_model_->projectTrajectory(current_pose.pose, map_frame_, odom_msg_.twist.twist.linear.x,
                                         drive_msg_.drive.steering_angle, delta_t_, t_max_);

    visualizeProjectedTrajectory(projected_trajectory);

    std_msgs::Float64 ttc;
    ttc.data = std::numeric_limits<double>::max();

    for (int i = 0; i < projected_trajectory.poses.size(); ++i)
    {
      if (collision_checker_->checkCollision(projected_trajectory.poses.at(i)))
      {
        ttc.data = i * delta_t_;
        break;
      }
    }

    time_to_collision_pub_.publish(ttc);
    ROS_WARN_COND(ttc.data < std::numeric_limits<double>::max(), "Time to collision: %.1f", ttc.data);
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }
  catch (const std::runtime_error& ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }
}

void CollisionWarningSystem::odomCallback(const nav_msgs::Odometry& odom_msg)
{
  odom_msg_ = odom_msg;
}

void CollisionWarningSystem::driveCallback(const ackermann_msgs::AckermannDriveStamped& drive_msg)
{
  drive_msg_ = drive_msg;
}

void CollisionWarningSystem::visualizeProjectedTrajectory(const nav_msgs::Path& path)
{
  visualization_msgs::Marker projected_marker;
  projected_marker.header.stamp = path.header.stamp;
  projected_marker.header.frame_id = path.header.frame_id;
  projected_marker.action = visualization_msgs::Marker::ADD;
  projected_marker.type = visualization_msgs::Marker::LINE_STRIP;
  projected_marker.lifetime = ros::Duration(0.1);
  projected_marker.pose.orientation.w = 1;
  projected_marker.points.reserve(path.poses.size() * 2);
  projected_marker.scale.x = 0.01;
  projected_marker.color.r = 0;
  projected_marker.color.g = 1;
  projected_marker.color.b = 0;
  projected_marker.color.a = 1;

  std::vector<geometry_msgs::Point> left_side;
  std::vector<geometry_msgs::Point> right_side;
  left_side.reserve(path.poses.size());
  right_side.reserve(path.poses.size());

  double half_width = vehicle_width_ / 2;

  for (const auto& pose_stamped : path.poses)
  {
    double yaw = TF2Wrapper::yawFromQuat(pose_stamped.pose.orientation);
    geometry_msgs::Point base = pose_stamped.pose.position;
    geometry_msgs::Point left;
    left.x = base.x + half_width * cos(yaw + M_PI_2);
    left.y = base.y + half_width * sin(yaw + M_PI_2);
    left_side.push_back(left);

    geometry_msgs::Point right;
    right.x = base.x - half_width * cos(yaw + M_PI_2);
    right.y = base.y - half_width * sin(yaw + M_PI_2);
    right_side.push_back(right);
  }

  double x_offset;
  double y_offset;
  double end_yaw = TF2Wrapper::yawFromQuat(path.poses.back().pose.orientation);

  if (!(odom_msg_.twist.twist.linear.x < 0))
  {
    double dist_to_front = base_link_to_center_dist_ + vehicle_length_ / 2;

    x_offset = dist_to_front * cos(end_yaw);
    y_offset = dist_to_front * sin(end_yaw);
  }
  else
  {
    x_offset = -base_link_to_center_dist_ * cos(end_yaw);
    y_offset = -base_link_to_center_dist_ * sin(end_yaw);
  }

  geometry_msgs::Point bumper_left;
  bumper_left.x = left_side.back().x + x_offset;
  bumper_left.y = left_side.back().y + y_offset;
  left_side.push_back(bumper_left);

  geometry_msgs::Point bumper_right;
  bumper_right.x = right_side.back().x + x_offset;
  bumper_right.y = right_side.back().y + y_offset;
  right_side.push_back(bumper_right);

  projected_marker.points.insert(projected_marker.points.begin(), left_side.begin(), left_side.end());
  projected_marker.points.insert(projected_marker.points.end(), right_side.rbegin(), right_side.rend());

  viz_pub_.publish(projected_marker);
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
