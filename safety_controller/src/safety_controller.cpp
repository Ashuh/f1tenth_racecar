#include <string>

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>

#include "safety_controller/safety_controller.h"

namespace f1tenth_racecar
{
namespace safety
{
SafetyController::SafetyController()
{
  ros::NodeHandle private_nh("~");

  std::string odom_topic;
  std::string time_to_collision_topic;
  std::string selected_drive_topic;
  std::string safe_drive_topic;

  getParam("odom_topic", odom_topic);
  getParam("time_to_collision_topic", time_to_collision_topic);
  getParam("selected_drive_topic", selected_drive_topic);
  getParam("safe_drive_topic", safe_drive_topic);

  odom_sub_ = nh_.subscribe(odom_topic, 1, &SafetyController::odomCallback, this);
  time_to_collision_sub_ = nh_.subscribe(time_to_collision_topic, 1, &SafetyController::timeToCollisionCallback, this);
  selected_drive_sub_ = nh_.subscribe(selected_drive_topic, 1, &SafetyController::driveCallback, this);

  safe_drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(safe_drive_topic, 1);
}

void SafetyController::driveCallback(const ackermann_msgs::AckermannDriveStamped drive_msg)
{
  ackermann_msgs::AckermannDriveStamped safe_drive_msg = drive_msg;

  if (time_to_collision_ < 0.5 && abs(current_velocity_) > 0.2)
  {
    safe_drive_msg.drive.speed = 0;
  }

  safe_drive_pub_.publish(safe_drive_msg);
}

void SafetyController::timeToCollisionCallback(const std_msgs::Float64 time_to_collision_msg)
{
  time_to_collision_ = time_to_collision_msg.data;
}

void SafetyController::odomCallback(const nav_msgs::Odometry odom_msg)
{
  current_velocity_ = odom_msg.twist.twist.linear.x;
}

template <typename T>
void SafetyController::getParam(const std::string& key, T& result)
{
  ros::NodeHandle private_nh("~");

  std::string found;
  if (private_nh.searchParam(key, found))
  {
    private_nh.getParam(found, result);
  }
  else
  {
    ROS_FATAL("[Safety Controller] Parameter [%s] not found, shutting down", key.c_str());
    ros::shutdown();
  }
}
}  // namespace safety
}  // namespace f1tenth_racecar

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "safety_system");
  f1tenth_racecar::safety::SafetyController safety_controller;
  ros::spin();
  return 0;
}
