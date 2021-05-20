#include <algorithm>
#include <string>

#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include "ds4_driver/Feedback.h"
#include "ds4_driver/Status.h"
#include "ds4_controller/ds4_controller.h"

namespace f1tenth_racecar
{
namespace utils
{
DS4Controller::DS4Controller()
{
  ros::NodeHandle private_nh("~");

  std::string feedback_topic;
  std::string drive_topic;

  ROS_ASSERT(private_nh.getParam("status_topic", status_topic_));
  ROS_ASSERT(private_nh.getParam("feedback_topic", feedback_topic));
  ROS_ASSERT(private_nh.getParam("drive_topic", drive_topic));
  ROS_ASSERT(private_nh.getParam("timeout", timeout_));
  ROS_ASSERT(private_nh.getParam("max_steering_angle", max_steering_angle_));
  ROS_ASSERT(private_nh.getParam("max_speed", max_speed_));

  status_sub_ = nh_.subscribe(status_topic_, 1, &DS4Controller::statusCallback, this);
  drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1);
  feedback_pub_ = nh_.advertise<ds4_driver::Feedback>(feedback_topic, 1);

  waitForConnection();

  timer_ = nh_.createTimer(ros::Duration(0.1), &DS4Controller::timerCallback, this);
}

void DS4Controller::statusCallback(const ds4_driver::Status status_msg)
{
  last_message_time_ = ros::Time::now();
  battery_percentage_ = status_msg.battery_percentage;
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

void DS4Controller::timerCallback(const ros::TimerEvent& timer_event)
{
  checkConnection();

  ds4_driver::Feedback feedback_msg;
  feedback_msg.set_led = true;
  batteryToRGB(feedback_msg.led_r, feedback_msg.led_g, feedback_msg.led_b);

  feedback_pub_.publish(feedback_msg);
}

void DS4Controller::checkConnection()
{
  if ((ros::Time::now() - last_message_time_).toSec() > timeout_)
  {
    ROS_ERROR("[DS4 Controller] Lost connection to DualShock 4");
    waitForConnection();
  }
}

void DS4Controller::waitForConnection()
{
  while (status_sub_.getNumPublishers() == 0)
  {
    ROS_WARN_THROTTLE(1, "[DS4 Controller] Waiting for DS4 Driver");
  }

  ROS_INFO("[DS4 Controller] DS4 Driver Started");

  while (ros::topic::waitForMessage<ds4_driver::Status>(status_topic_, ros::Duration(0.1)) == NULL)
  {
    ROS_WARN_THROTTLE(1, "[DS4 Controller] Waiting for Connection to DualShock 4");
  }

  ROS_INFO("[DS4 Controller] Connection success");

  ds4_driver::Feedback feedback_msg;
  feedback_msg.set_rumble = true;
  feedback_msg.rumble_duration = 0.5;
  feedback_msg.rumble_small = 1;
  feedback_pub_.publish(feedback_msg);

  ros::Duration(0.5).sleep();
}

void DS4Controller::batteryToRGB(float& r, float& g, float& b)
{
  r = std::min(2.0 * (1 - battery_percentage_), 1.0);
  g = std::min(2.0 * battery_percentage_, 1.0);
  b = 0;
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
