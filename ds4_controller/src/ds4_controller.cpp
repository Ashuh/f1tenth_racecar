#include <string>

#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include "ds4_driver/Status.h"
#include "ds4_controller/ds4_controller.h"

namespace f1tenth_racecar
{
namespace utils
{
DS4Controller::DS4Controller()
{
  ros::NodeHandle private_nh("~");

  std::string status_topic;
  std::string drive_topic;

  ROS_ASSERT(private_nh.getParam("status_topic", status_topic));
  ROS_ASSERT(private_nh.getParam("drive_topic", drive_topic));
  ROS_ASSERT(private_nh.getParam("max_steering_angle", max_steering_angle_));
  ROS_ASSERT(private_nh.getParam("max_speed", max_speed_));

  status_sub_ = nh_.subscribe(status_topic, 1, &DS4Controller::statusCallback, this);
  drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1);
}

void DS4Controller::statusCallback(const ds4_driver::Status status_msg)
{
  publishDriveMsg(status_msg.axis_right_y, status_msg.axis_left_x);
}

void DS4Controller::publishDriveMsg(const double throttle_axis, const double steering_axis)
{
  ackermann_msgs::AckermannDriveStamped drive_msg;
  drive_msg.header.stamp = ros::Time::now();
  drive_msg.drive.speed = throttle_axis * max_speed_;
  drive_msg.drive.steering_angle = steering_axis * max_steering_angle_;

  drive_pub_.publish(drive_msg);
}
}  // namespace utils
}  // namespace f1tenth_racecar

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "ds4_controller");
  f1tenth_racecar::utils::DS4Controller ds4_controller;
  ros::spin();
  return 0;
}
