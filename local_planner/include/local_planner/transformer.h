#ifndef LOCAL_PLANNER_TRANSFORMER_H
#define LOCAL_PLANNER_TRANSFORMER_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include "local_planner/trajectory.h"

class Transformer
{
public:
  static geometry_msgs::Pose transform(const geometry_msgs::Pose& pose, const std::string& target_frame,
                                       const std::string& source_frame);

  static geometry_msgs::PoseStamped transform(const geometry_msgs::PoseStamped& pose, const std::string& target_frame);

  static Path transform(const Path& path, const std::string& target_frame);

  static geometry_msgs::Point transform(const geometry_msgs::Pose& ref_pose, const geometry_msgs::Point& target_point);

  static geometry_msgs::Pose transform(const geometry_msgs::Pose& ref_pose, const geometry_msgs::Pose& target_pose);

  static geometry_msgs::Quaternion quatFromYaw(const double yaw);

  static double yawFromQuat(const geometry_msgs::Quaternion quat);

private:
  static tf2::Transform poseToTransform(const geometry_msgs::Pose& pose);

  static tf2_ros::Buffer& buffer();
};

#endif  // LOCAL_PLANNER_TRANSFORMER_H
