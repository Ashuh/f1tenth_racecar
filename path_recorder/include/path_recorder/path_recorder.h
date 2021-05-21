#ifndef PATH_RECORDER_PATH_RECORDER_H
#define PATH_RECORDER_PATH_RECORDER_H

#include <fstream>
#include <string>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include "path_recorder/csv_io.h"

namespace f1tenth_racecar
{
namespace utils
{
class PathRecorder
{
private:
  enum class Mode
  {
    Publish = 0,
    Record = 1
  };

  ros::NodeHandle nh_;

  ros::Subscriber odom_sub_;
  ros::Publisher path_pub_;

  nav_msgs::Path path_;
  geometry_msgs::Pose last_pose_;

  CsvIO csv_io_;
  double interval_;

public:
  PathRecorder();

  void odomCallback(const nav_msgs::Odometry odom_msg);

  double dist(const geometry_msgs::Point point_1, const geometry_msgs::Point point_2);

  void writePose(const geometry_msgs::Pose pose);
};
}  // namespace utils
}  // namespace f1tenth_racecar

#endif  // PATH_RECORDER_PATH_RECORDER_H
