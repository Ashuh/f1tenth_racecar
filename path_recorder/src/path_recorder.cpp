#include <fstream>
#include <string>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include "path_recorder/path_recorder.h"
#include "path_recorder/csv_io.h"

namespace f1tenth_racecar
{
namespace utils
{
PathRecorder::PathRecorder()
{
  ros::NodeHandle private_nh("~");

  int mode;
  std::string file_name;
  std::string path_topic;

  ROS_ASSERT(private_nh.getParam("mode", mode));
  ROS_ASSERT(private_nh.getParam("file_name", file_name) && (file_name.find(".csv") != std::string::npos));
  ROS_ASSERT(private_nh.getParam("path_topic", path_topic));
  ROS_ASSERT(private_nh.getParam("interval", interval_));

  path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic, 1, true);

  if (mode == static_cast<int>(Mode::Record))
  {
    std::string odom_topic;
    ROS_ASSERT(private_nh.getParam("odom_topic", odom_topic));
    odom_sub_ = nh_.subscribe(odom_topic, 1, &PathRecorder::odomCallback, this);
    csv_io_.write(file_name);
    ROS_INFO_STREAM("[Path Recorder] Writing to " << csv_io_.filePath());

    nav_msgs::Odometry odom_msg = *ros::topic::waitForMessage<nav_msgs::Odometry>(odom_topic);
    path_.header.frame_id = odom_msg.header.frame_id;

    csv_io_ << "frame_id" << odom_msg.header.frame_id << "\n";

    csv_io_ << "x"
            << "y"
            << "q_w"
            << "q_x"
            << "q_y"
            << "q_z"
            << "\n";
  }
  else if (mode == static_cast<int>(Mode::Publish))
  {
    if (!csv_io_.read(file_name))
    {
      ROS_ERROR("[Path Recorder] Could not open file");
      ros::shutdown();
    }

    ROS_INFO_STREAM("[Path Recorder] Opened " << csv_io_.filePath());

    std::string dummy;

    csv_io_ >> dummy >> path_.header.frame_id;
    csv_io_ >> dummy >> dummy >> dummy >> dummy >> dummy >> dummy;

    geometry_msgs::PoseStamped pose_stamped;

    while (csv_io_ >> pose_stamped.pose.position.x >> pose_stamped.pose.position.y >> pose_stamped.pose.orientation.w >>
           pose_stamped.pose.orientation.x >> pose_stamped.pose.orientation.y >> pose_stamped.pose.orientation.z)
    {
      ROS_DEBUG_STREAM("[Path Recorder] Read pose: "
                       << std::setprecision(2) << pose_stamped.pose.position.x << " " << pose_stamped.pose.position.y
                       << " " << pose_stamped.pose.orientation.w << " " << pose_stamped.pose.orientation.x << " "
                       << pose_stamped.pose.orientation.y << " " << pose_stamped.pose.orientation.z);
      path_.poses.push_back(pose_stamped);
    }

    ROS_INFO("[Path Recorder] Done reading file, publishing path");
    path_pub_.publish(path_);
    ros::shutdown();
  }
}

void PathRecorder::odomCallback(const nav_msgs::Odometry odom_msg)
{
  if (dist(odom_msg.pose.pose.position, last_pose_.position) >= interval_)
  {
    ROS_INFO("[Path Recorder] Writing pose");

    writePose(odom_msg.pose.pose);
    last_pose_ = odom_msg.pose.pose;

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = odom_msg.header;
    pose_stamped.pose = odom_msg.pose.pose;
    path_.poses.push_back(pose_stamped);
    path_pub_.publish(path_);
  }
}

double PathRecorder::dist(const geometry_msgs::Point point_1, const geometry_msgs::Point point_2)
{
  double d_x = point_1.x - point_2.x;
  double d_y = point_1.y - point_2.y;

  return sqrt(pow(d_x, 2) + pow(d_y, 2));
}

void PathRecorder::writePose(const geometry_msgs::Pose pose)
{
  csv_io_ << pose.position.x << pose.position.y << pose.orientation.w << pose.orientation.x << pose.orientation.y
          << pose.orientation.z << "\n";
}
}  // namespace utils
}  // namespace f1tenth_racecar

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "safety_system");
  f1tenth_racecar::utils::PathRecorder path_recorder;
  ros::spin();
  return 0;
}
