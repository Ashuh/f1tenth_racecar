#include <string>
#include <vector>

#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
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
  std::string trajectory_topic = "trajectory";
  std::string vehicle_footprints_viz_topic = "vehicle_footprints_viz";
  std::string collision_viz_topic = "collision_viz";

  double wheel_base = 0.3;

  model_ = BicycleModel(wheel_base);
  collision_checker_ = new CollisionChecker(1, 1, 0.5);

  odom_sub_ = nh_.subscribe(odom_topic, 1, &CollisionWarningSystem::odomCallback, this);
  drive_sub_ = nh_.subscribe(drive_topic, 1, &CollisionWarningSystem::driveCallback, this);
  obstacle_sub_ = nh_.subscribe(obstacle_topic, 1, &CollisionWarningSystem::obstacleCallback, this);

  trajectory_pub_ = nh_.advertise<nav_msgs::Path>(trajectory_topic, 1);
  vehicle_footprints_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(vehicle_footprints_viz_topic, 1);
  collision_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(collision_viz_topic, 1);
}

void CollisionWarningSystem::timerCallback(const ros::TimerEvent& timer_event)
{
  geometry_msgs::TransformStamped map_to_obstacle_frame;

  try
  {
    map_to_obstacle_frame = tf_buffer.lookupTransform("laser", "map", ros::Time(0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }

  geometry_msgs::Pose transformed_pose;
  tf2::doTransform(odom_msg_.pose.pose, transformed_pose, map_to_obstacle_frame);
  nav_msgs::Path projected_trajectory = model_.projectTrajectory(
      transformed_pose, "laser", odom_msg_.twist.twist.linear.x, drive_msg_.drive.steering_angle, delta_t_, steps_);

  trajectory_pub_.publish(projected_trajectory);

  std::vector<geometry_msgs::PolygonStamped> footprints;
  int collision_index;

  for (int i = 0; i < projected_trajectory.poses.size(); ++i)
  {
    geometry_msgs::PolygonStamped footprint;

    bool collision = collision_checker_->collisionCheck(projected_trajectory.poses.at(i).pose, obstacles_msg_);
    collision_checker_->getCollisionInfo(footprint, collision_index);

    footprints.push_back(footprint);

    if (collision)
    {
      break;
    }
  }

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

    for (int j = 0; j < obstacle.footprint.points.size() + 1; j++)
    {
      geometry_msgs::Point p;
      p.x = obstacle.footprint.points.at(j % obstacle.footprint.points.size()).x;
      p.y = obstacle.footprint.points.at(j % obstacle.footprint.points.size()).y;
      obstacle_marker.points.push_back(p);
    }

    obstacle_marker.action = visualization_msgs::Marker::ADD;
    obstacle_marker.type = visualization_msgs::Marker::LINE_STRIP;
    obstacle_marker.id = i;
    obstacle_marker.lifetime = ros::Duration(0.1);
    obstacle_marker.header.frame_id = "laser";

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

void CollisionWarningSystem::visualizeVehicleFootprints(const std::vector<geometry_msgs::PolygonStamped> footprints)
{
  visualization_msgs::MarkerArray footprint_markers;

  for (int i = 0; i < footprints.size(); ++i)
  {
    visualization_msgs::Marker footprint_marker;
    geometry_msgs::Polygon footprint = footprints.at(i).polygon;

    for (int j = 0; j < footprint.points.size() + 1; j++)
    {
      geometry_msgs::Point p;
      p.x = footprint.points.at(j % footprint.points.size()).x;
      p.y = footprint.points.at(j % footprint.points.size()).y;
      footprint_marker.points.push_back(p);
    }

    footprint_marker.action = visualization_msgs::Marker::ADD;
    footprint_marker.type = visualization_msgs::Marker::LINE_STRIP;
    footprint_marker.id = i;
    footprint_marker.lifetime = ros::Duration(0.1);
    footprint_marker.header.frame_id = "laser";

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
