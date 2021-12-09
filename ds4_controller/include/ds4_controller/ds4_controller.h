#ifndef DS4_CONTROLLER_DS4_CONTROLLER_H
#define DS4_CONTROLLER_DS4_CONTROLLER_H

#include <string>

#include <ros/ros.h>

#include "ds4_driver/Status.h"

class DS4Controller
{
private:
  ros::NodeHandle nh_;

  ros::Timer timer_;

  ros::Subscriber status_sub_;

  ros::Publisher feedback_pub_;
  ros::Publisher drive_pub_;
  ros::Publisher drive_mode_pub_;

  ros::Time last_message_time_;
  double timeout_;

  double battery_percentage_;

  double max_steering_angle_;
  double max_speed_;

  void statusCallback(const ds4_driver::Status status_msg);

  void publishDriveMsg(const double throttle_axis, const double steering_axis);

  void timerCallback(const ros::TimerEvent& timer_event);

  void checkConnection();

  void waitForConnection();

  void batteryToRGB(float& r, float& g, float& b);

  template <typename T>
  void getParam(const std::string& key, T& result);

public:
  DS4Controller();
};

#endif  // DS4_CONTROLLER_DS4_CONTROLLER_H
