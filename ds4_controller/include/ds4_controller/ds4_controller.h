#ifndef DS4_CONTROLLER_DS4_CONTROLLER_H
#define DS4_CONTROLLER_DS4_CONTROLLER_H

#include <ros/ros.h>

#include "ds4_driver/Status.h"

namespace f1tenth_racecar
{
namespace utils
{
class DS4Controller
{
private:
  ros::NodeHandle nh_;

  ros::Timer timer_;

  ros::Subscriber status_sub_;

  ros::Publisher feedback_pub_;
  ros::Publisher drive_pub_;

  double max_steering_angle_;
  double max_speed_;

  double battery_percentage_;

  void statusCallback(const ds4_driver::Status status_msg);

  void publishDriveMsg(const double throttle_axis, const double steering_axis);

  void publishFeedbackMsg(const ros::TimerEvent& timer_event);

  void waitForConnection();

  void batteryToRGB(float& r, float& g, float& b);

public:
  DS4Controller();
};
}  // namespace utils
}  // namespace f1tenth_racecar

#endif  // DS4_CONTROLLER_DS4_CONTROLLER_H
