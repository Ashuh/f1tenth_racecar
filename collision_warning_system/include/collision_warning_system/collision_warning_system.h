#ifndef COLLISION_WARNING_SYSTEM_COLLISION_WARNING_SYSTEM_H
#define COLLISION_WARNING_SYSTEM_COLLISION_WARNING_SYSTEM_H

#include <vector>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <tf2_ros/transform_listener.h>

#include "f1tenth_msgs/ObstacleArray.h"
#include "collision_warning_system/bicycle_model.h"
#include "collision_warning_system/bicycle_state.h"
#include "collision_warning_system/collision_checker.h"

namespace f1tenth_racecar
{
namespace safety
{
class CollisionWarningSystem
{
private:
  ros::NodeHandle nh_;
  ros::Timer timer_;

  // Subscribers
  ros::Subscriber odom_sub_;
  ros::Subscriber drive_sub_;
  ros::Subscriber obstacle_sub_;

  // Publishers
  ros::Publisher trajectory_pub_;
  ros::Publisher vehicle_footprints_pub_;
  ros::Publisher collision_viz_pub_;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

  BicycleModel model_;
  CollisionChecker* collision_checker_;

  double delta_t_ = 0.5;
  double steps_ = 10;

  nav_msgs::Odometry odom_msg_;
  ackermann_msgs::AckermannDriveStamped drive_msg_;
  f1tenth_msgs::ObstacleArray obstacles_msg_;

  void timerCallback(const ros::TimerEvent& timer_event);
  void odomCallback(const nav_msgs::Odometry odom_msg);
  void driveCallback(const ackermann_msgs::AckermannDriveStamped drive_msg);
  void obstacleCallback(const f1tenth_msgs::ObstacleArray obstacles);
  void visualizeCollisions(double collision_index);
  void visualizeVehicleFootprints(const std::vector<geometry_msgs::PolygonStamped> footprints);

public:
  CollisionWarningSystem();
};
}  // namespace safety
}  // namespace f1tenth_racecar
#endif  // COLLISION_WARNING_SYSTEM_COLLISION_WARNING_SYSTEM_H
