#ifndef F1TENTH_TELEOP_DS4_F1TENTH_TELEOP_DS4_H
#define F1TENTH_TELEOP_DS4_F1TENTH_TELEOP_DS4_H

#include <string>
#include <vector>

#include <ros/ros.h>

#include "ds4_driver/Status.h"

class TeleopDS4
{
public:
  TeleopDS4();

private:
  enum Mode
  {
    CRAWL = 0,
    NORMAL = 1,
    RACE = 2
  };

  ros::NodeHandle nh_;
  ros::Timer timer_;

  ros::Subscriber status_sub_;
  ros::Publisher feedback_pub_;
  ros::Publisher drive_pub_;
  ros::Publisher drive_mode_pub_;

  ds4_driver::Status prev_status_;

  double timeout_;
  std::vector<double> max_speeds_;
  double min_speed_;
  double max_steering_angle_;
  double battery_percentage_;
  Mode mode_;

  void statusCallback(const ds4_driver::Status& status_msg);

  void publishDriveMsg(const double throttle_axis, const double steering_axis);

  void timerCallback(const ros::TimerEvent& timer_event);

  void checkConnection();

  void waitForConnection();

  void batteryToRGB(float& r, float& g, float& b);

  bool buttonIsPressed(const ds4_driver::Status& status, int32_t ds4_driver::Status::*button) const;
};

#endif  // F1TENTH_TELEOP_DS4_F1TENTH_TELEOP_DS4_H
