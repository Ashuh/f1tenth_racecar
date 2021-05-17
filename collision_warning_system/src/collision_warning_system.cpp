#include <limits>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "f1tenth_msgs/ObstacleArray.h"
#include "collision_warning_system/bicycle_model.h"
#include "collision_warning_system/collision_checker.h"
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
  std::string obstacle_topic = "obstacles";
  std::string time_to_collision_topic = "time_to_collision";
  std::string trajectory_topic = "trajectory";
  std::string vehicle_footprints_viz_topic = "vehicle_footprints_viz";
  std::string collision_viz_topic = "collision_viz";

  double wheel_base = 0.3;

  biycle_model_ = new BicycleModel(wheel_base);
  collision_checker_ = new CollisionChecker(1, 1, 0.5);

  odom_sub_ = nh_.subscribe(odom_topic, 1, &CollisionWarningSystem::odomCallback, this);
  drive_sub_ = nh_.subscribe(drive_topic, 1, &CollisionWarningSystem::driveCallback, this);
  obstacle_sub_ = nh_.subscribe(obstacle_topic, 1, &CollisionWarningSystem::obstacleCallback, this);

  time_to_collision_pub_ = nh_.advertise<std_msgs::Float64>(time_to_collision_topic, 1);
  trajectory_pub_ = nh_.advertise<nav_msgs::Path>(trajectory_topic, 1);
  vehicle_footprints_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(vehicle_footprints_viz_topic, 1);
  collision_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(collision_viz_topic, 1);
}

void CollisionWarningSystem::timerCallback(const ros::TimerEvent& timer_event)
{
  geometry_msgs::TransformStamped odom_to_obstacle_frame;

  try
  {
    odom_to_obstacle_frame = tf_buffer.lookupTransform(obstacle_frame_, odom_frame_, ros::Time(0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }

  geometry_msgs::Pose transformed_pose;
  tf2::doTransform(odom_msg_.pose.pose, transformed_pose, odom_to_obstacle_frame);
  nav_msgs::Path projected_trajectory =
      biycle_model_->projectTrajectory(transformed_pose, obstacle_frame_, odom_msg_.twist.twist.linear.x,
                                       drive_msg_.drive.steering_angle, delta_t_, steps_);

  trajectory_pub_.publish(projected_trajectory);

  std::vector<f1tenth_msgs::RectangleStamped> footprints;
  int collision_index = -1;
  std_msgs::Float64 time_to_collision_msg;
  time_to_collision_msg.data = std::numeric_limits<double>::max();

  for (int i = 0; i < projected_trajectory.poses.size(); ++i)
  {
    f1tenth_msgs::RectangleStamped footprint;

    bool collision = collision_checker_->collisionCheck(projected_trajectory.poses.at(i), obstacles_msg_);
    collision_checker_->getCollisionInfo(footprint, collision_index);

    footprints.push_back(footprint);

    if (collision)
    {
      time_to_collision_msg.data = i * delta_t_;
      break;
    }
  }

  time_to_collision_pub_.publish(time_to_collision_msg);
  visualizeVehicleFootprints(footprints);
  visualizeCollisions(collision_index);
}

void CollisionWarningSystem::odomCallback(const nav_msgs::Odometry odom_msg)
{
  odom_msg_ = odom_msg;
}

void CollisionWarningSystem::driveCallback(const ackermann_msgs::AckermannDriveStamped drive_msg)
{
  drive_msg_ = drive_msg;
}

void CollisionWarningSystem::obstacleCallback(const f1tenth_msgs::ObstacleArray obstacles_msg)
{
  obstacles_msg_ = obstacles_msg;
}

void CollisionWarningSystem::visualizeCollisions(double collision_index)
{
  visualization_msgs::MarkerArray obstacle_markers;

  for (int i = 0; i < obstacles_msg_.obstacles.size(); ++i)
  {
    f1tenth_msgs::Obstacle obstacle = obstacles_msg_.obstacles.at(i);
    visualization_msgs::Marker obstacle_marker;

    obstacle_marker.points = { obstacle.footprint.rectangle.a, obstacle.footprint.rectangle.b,
                               obstacle.footprint.rectangle.c, obstacle.footprint.rectangle.d,
                               obstacle.footprint.rectangle.a };

    obstacle_marker.action = visualization_msgs::Marker::ADD;
    obstacle_marker.type = visualization_msgs::Marker::LINE_STRIP;
    obstacle_marker.id = i;
    obstacle_marker.lifetime = ros::Duration(0.1);
    obstacle_marker.header.frame_id = obstacle.header.frame_id;

    obstacle_marker.scale.x = 0.05;
    obstacle_marker.scale.y = 0.05;
    obstacle_marker.scale.z = 0.05;

    if (i == collision_index)
    {
      obstacle_marker.color.r = 1;
      obstacle_marker.color.g = 0;
      obstacle_marker.color.b = 0;
      obstacle_marker.color.a = 1;
    }
    else
    {
      obstacle_marker.color.r = 0;
      obstacle_marker.color.g = 1;
      obstacle_marker.color.b = 0;
      obstacle_marker.color.a = 1;
    }

    obstacle_markers.markers.push_back(obstacle_marker);
  }

  collision_viz_pub_.publish(obstacle_markers);
}

void CollisionWarningSystem::visualizeVehicleFootprints(const std::vector<f1tenth_msgs::RectangleStamped> footprints)
{
  visualization_msgs::MarkerArray footprint_markers;

  for (int i = 0; i < footprints.size(); ++i)
  {
    visualization_msgs::Marker footprint_marker;
    f1tenth_msgs::RectangleStamped footprint = footprints.at(i);

    footprint_marker.points = { footprint.rectangle.a, footprint.rectangle.b, footprint.rectangle.c,
                                footprint.rectangle.d, footprint.rectangle.a };

    footprint_marker.action = visualization_msgs::Marker::ADD;
    footprint_marker.type = visualization_msgs::Marker::LINE_STRIP;
    footprint_marker.id = i;
    footprint_marker.lifetime = ros::Duration(0.1);
    footprint_marker.header.frame_id = footprint.header.frame_id;

    footprint_marker.scale.x = 0.05;
    footprint_marker.scale.y = 0.05;
    footprint_marker.scale.z = 0.05;

    footprint_marker.color.r = 0;
    footprint_marker.color.g = 1;
    footprint_marker.color.b = 0;
    footprint_marker.color.a = 1;

    footprint_markers.markers.push_back(footprint_marker);
  }

  vehicle_footprints_pub_.publish(footprint_markers);
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
