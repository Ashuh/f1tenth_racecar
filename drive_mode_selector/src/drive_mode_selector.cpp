#include <string>

#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include "f1tenth_msgs/DriveMode.h"
#include "drive_mode_selector/drive_mode_selector.h"

namespace f1tenth_racecar
{
DriveModeSelector::DriveModeSelector()
{
  ros::NodeHandle private_nh("~");

  std::string drive_mode_topic;
  std::string manual_drive_topic;
  std::string auto_drive_topic;
  std::string selected_drive_topic;

  ROS_ASSERT(private_nh.getParam("drive_mode_topic", drive_mode_topic));
  ROS_ASSERT(private_nh.getParam("manual_drive_topic", manual_drive_topic));
  ROS_ASSERT(private_nh.getParam("auto_drive_topic", auto_drive_topic));
  ROS_ASSERT(private_nh.getParam("selected_drive_topic", selected_drive_topic));

  drive_mode_sub_ = nh_.subscribe(drive_mode_topic, 1, &DriveModeSelector::driveModeCallback, this);
  manual_drive_sub_ = nh_.subscribe(manual_drive_topic, 1, &DriveModeSelector::manualDriveCallback, this);
  auto_drive_sub_ = nh_.subscribe(auto_drive_topic, 1, &DriveModeSelector::autoDriveCallback, this);

  selected_drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(selected_drive_topic, 1);

  drive_mode_ = Mode::PARK;
}

void DriveModeSelector::driveModeCallback(const f1tenth_msgs::DriveMode drive_mode_msg)
{
  switch (drive_mode_msg.drive_mode)
  {
    case f1tenth_msgs::DriveMode::PARK:
      if (drive_mode_ != Mode::PARK)
      {
        drive_mode_ = Mode::PARK;
        ackermann_msgs::AckermannDriveStamped stop_drive_msg;
        selected_drive_pub_.publish(stop_drive_msg);
        ROS_INFO("[Drive Mode Selector] Drive mode set to Park");
      }
      break;
    case f1tenth_msgs::DriveMode::MANUAL:
      if (drive_mode_ != Mode::MANUAL)
      {
        drive_mode_ = Mode::MANUAL;
        ROS_INFO("[Drive Mode Selector] Drive mode set to Manual");
      }
      break;
    case f1tenth_msgs::DriveMode::AUTO:
      if (drive_mode_ != Mode::AUTO)
      {
        drive_mode_ = Mode::AUTO;
        ROS_INFO("[Drive Mode Selector] Drive mode set to Auto");
      }
      break;
    default:
      break;
  }
}

void DriveModeSelector::manualDriveCallback(const ackermann_msgs::AckermannDriveStamped drive_msg)
{
  if (drive_mode_ == Mode::MANUAL)
  {
    selected_drive_pub_.publish(drive_msg);
  }
}

void DriveModeSelector::autoDriveCallback(const ackermann_msgs::AckermannDriveStamped drive_msg)
{
  if (drive_mode_ == Mode::AUTO)
  {
    selected_drive_pub_.publish(drive_msg);
  }
}
}  // namespace f1tenth_racecar

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "drive_controller");
  f1tenth_racecar::DriveModeSelector drive_mode_selector;
  ros::spin();
  return 0;
}
