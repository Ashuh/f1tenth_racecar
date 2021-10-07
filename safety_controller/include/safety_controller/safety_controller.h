#ifndef SAFETY_CONTROLLER_SAFETY_CONTROLLER_H
#define SAFETY_CONTROLLER_SAFETY_CONTROLLER_H

#include <string>

#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>

namespace f1tenth_racecar
{
namespace safety
{
class SafetyController
{
private:
  ros::NodeHandle nh_;

  ros::Subscriber odom_sub_;
  ros::Subscriber time_to_collision_sub_;
  ros::Subscriber selected_drive_sub_;

  ros::Publisher safe_drive_pub_;

  double time_to_collision_;
  double current_velocity_;

  template <typename T>
  void getParam(const std::string& key, T& result);

public:
  SafetyController();

  void driveCallback(const ackermann_msgs::AckermannDriveStamped drive_msg);

  void timeToCollisionCallback(const std_msgs::Float64 time_to_collision_msg);

  void odomCallback(const nav_msgs::Odometry odom_msg);
};
}  // namespace safety
}  // namespace f1tenth_racecar

#endif  // SAFETY_CONTROLLER_SAFETY_CONTROLLER_H
