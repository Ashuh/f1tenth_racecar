#include "drive_controller/drive_controller.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "drive_controller");
  DriveController drive_controller;
  ros::spin();
  return 0;
}
