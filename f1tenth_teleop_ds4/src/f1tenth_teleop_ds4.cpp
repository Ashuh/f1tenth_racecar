#include <algorithm>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include "ds4_driver/Feedback.h"
#include "ds4_driver/Status.h"
#include "f1tenth_msgs/DriveMode.h"
#include "f1tenth_teleop_ds4/f1tenth_teleop_ds4.h"

TeleopDS4::TeleopDS4()
{
  mode_ = Mode::NORMAL;
  max_speeds_ = std::vector<double>(3);
  ros::NodeHandle private_nh("~");

  private_nh.getParam("timeout", timeout_);
  private_nh.getParam("min_speed", min_speed_);
  private_nh.getParam("max_speed_crawl", max_speeds_.at(0));
  private_nh.getParam("max_speed_normal", max_speeds_.at(1));
  private_nh.getParam("max_speed_race", max_speeds_.at(2));
  private_nh.getParam("max_steering_angle", max_steering_angle_);

  status_sub_ = nh_.subscribe("status", 1, &TeleopDS4::statusCallback, this);
  drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("drive_joy", 1);
  drive_mode_pub_ = nh_.advertise<f1tenth_msgs::DriveMode>("drive_mode", 1);
  feedback_pub_ = nh_.advertise<ds4_driver::Feedback>("set_feedback", 1);
  timer_ = nh_.createTimer(ros::Duration(0.1), &TeleopDS4::timerCallback, this);

  waitForConnection();
}

void TeleopDS4::statusCallback(const ds4_driver::Status status_msg)
{
  battery_percentage_ = std::min(status_msg.battery_percentage, 1.0f);  // Fix driver reporting value greater than 1

  f1tenth_msgs::DriveMode drive_mode_msg;

  if (status_msg.button_circle == 1)
  {
    drive_mode_msg.drive_mode = f1tenth_msgs::DriveMode::MANUAL;
    drive_mode_pub_.publish(drive_mode_msg);
  }
  else if (status_msg.button_triangle == 1)
  {
    drive_mode_msg.drive_mode = f1tenth_msgs::DriveMode::AUTO;
    drive_mode_pub_.publish(drive_mode_msg);
  }
  else if (status_msg.button_square == 1)
  {
    drive_mode_msg.drive_mode = f1tenth_msgs::DriveMode::PARK;
    drive_mode_pub_.publish(drive_mode_msg);
  }

  if (buttonIsPressed(status_msg, &ds4_driver::Status::button_l1) && mode_ != Mode::CRAWL)
  {
    const int i = static_cast<int>(mode_) - 1;
    mode_ = static_cast<Mode>(i);
  }
  else if (buttonIsPressed(status_msg, &ds4_driver::Status::button_r1) && mode_ != Mode::RACE)
  {
    const int i = static_cast<int>(mode_) + 1;
    mode_ = static_cast<Mode>(i);
  }

  publishDriveMsg(status_msg.axis_right_y, status_msg.axis_left_x);
  prev_status_ = status_msg;
}

void TeleopDS4::publishDriveMsg(const double throttle_axis, const double steering_axis)
{
  ackermann_msgs::AckermannDriveStamped drive_msg;
  drive_msg.header.stamp = ros::Time::now();

  if (std::abs(throttle_axis) > 0)
  {
    bool reverse = throttle_axis < 0;
    drive_msg.drive.speed = throttle_axis * (max_speeds_.at(mode_) - min_speed_) + (reverse ? -1 : 1) * min_speed_;
  }
  else
  {
    drive_msg.drive.speed = 0;
  }

  drive_msg.drive.steering_angle = steering_axis * max_steering_angle_;
  drive_pub_.publish(drive_msg);
}

void TeleopDS4::timerCallback(const ros::TimerEvent& timer_event)
{
  checkConnection();

  ds4_driver::Feedback feedback_msg;
  feedback_msg.set_led = true;
  batteryToRGB(feedback_msg.led_r, feedback_msg.led_g, feedback_msg.led_b);

  feedback_pub_.publish(feedback_msg);
}

void TeleopDS4::checkConnection()
{
  if ((ros::Time::now() - prev_status_.header.stamp).toSec() > timeout_)
  {
    ROS_ERROR("Lost connection to DualShock 4");
    waitForConnection();
  }
}

void TeleopDS4::waitForConnection()
{
  while (ros::topic::waitForMessage<ds4_driver::Status>(status_sub_.getTopic(), ros::Duration(0.1)) == nullptr)
  {
    ROS_WARN_THROTTLE(1, "Waiting for Connection to DualShock 4");
  }

  ROS_INFO("Connected to DualShock 4");

  ds4_driver::Feedback feedback_msg;
  feedback_msg.set_rumble = true;
  feedback_msg.rumble_duration = 0.5;
  feedback_msg.rumble_small = 1;
  feedback_pub_.publish(feedback_msg);
  ros::Duration(0.5).sleep();
  prev_status_.header.stamp = ros::Time::now();
}

void TeleopDS4::batteryToRGB(float& r, float& g, float& b)
{
  r = std::min(2.0 * (1 - battery_percentage_), 1.0);
  g = std::min(2.0 * battery_percentage_, 1.0);
  b = 0;
}

bool TeleopDS4::buttonIsPressed(const ds4_driver::Status& status, int32_t ds4_driver::Status::*button) const
{
  return status.*button == 1 && prev_status_.*button == 0;
}
