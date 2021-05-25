#ifndef DRIVE_MODE_SELECTOR_DRIVE_MODE_SELECTOR_H
#define DRIVE_MODE_SELECTOR_DRIVE_MODE_SELECTOR_H

#include <ros/ros.h>

#include "f1tenth_msgs/DriveMode.h"

namespace f1tenth_racecar
{
class DriveModeSelector
{
private:
  enum class Mode
  {
    PARK,
    MANUAL,
    AUTO
  };

  ros::NodeHandle nh_;

  ros::Subscriber drive_mode_sub_;
  ros::Subscriber manual_drive_sub_;
  ros::Subscriber auto_drive_sub_;

  ros::Publisher selected_drive_pub_;

  Mode drive_mode_;

public:
  DriveModeSelector();

  void driveModeCallback(const f1tenth_msgs::DriveMode drive_mode_msg);

  void manualDriveCallback(const ackermann_msgs::AckermannDriveStamped drive_msg);

  void autoDriveCallback(const ackermann_msgs::AckermannDriveStamped drive_msg);
};
}  // namespace f1tenth_racecar

#endif  // DRIVE_MODE_SELECTOR_DRIVE_MODE_SELECTOR_H
