#include <ros/ros.h>

#include "f1tenth_teleop_ds4/f1tenth_teleop_ds4.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "f1tenth_teleop_ds4");
  TeleopDS4 teleop;
  ros::spin();
  return 0;
}
