#ifndef COLLISION_WARNING_SYSTEM_COLLISION_WARNING_SYSTEM_H
#define COLLISION_WARNING_SYSTEM_COLLISION_WARNING_SYSTEM_H

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <string>
#include <vector>

#include "collision_warning_system/bicycle_model.h"
#include "collision_warning_system/bicycle_state.h"
#include "costmap_generator/collision_checker.h"

class CollisionWarningSystem
{
public:
  CollisionWarningSystem();

private:
  ros::NodeHandle nh_;
  ros::Timer timer_;

  // Subscribers
  ros::Subscriber odom_sub_;
  ros::Subscriber drive_sub_;
  ros::Subscriber obstacle_sub_;

  // Publishers
  ros::Publisher time_to_collision_pub_;
  ros::Publisher viz_pub_;

  BicycleModel* biycle_model_;
  std::unique_ptr<CollisionChecker> collision_checker_;

  std::string map_frame_ = "map";

  nav_msgs::Odometry odom_msg_;
  ackermann_msgs::AckermannDriveStamped drive_msg_;

  double t_max_;
  double delta_t_;
  double vehicle_length_;
  double vehicle_width_;
  double base_link_to_center_dist_;

  void timerCallback(const ros::TimerEvent& timer_event);

  void odomCallback(const nav_msgs::Odometry& odom_msg);

  void driveCallback(const ackermann_msgs::AckermannDriveStamped& drive_msg);

  void visualizeProjectedTrajectory(const nav_msgs::Path& path);
};

#endif  // COLLISION_WARNING_SYSTEM_COLLISION_WARNING_SYSTEM_H
