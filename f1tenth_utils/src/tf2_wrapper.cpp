#include <string>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

#include "f1tenth_utils/tf2_wrapper.h"

geometry_msgs::TransformStamped TF2Wrapper::lookupTransform(const std::string& target_frame,
                                                            const std::string& source_frame, const ros::Time& time)
{
  return buffer().lookupTransform(target_frame, source_frame, time);
}

geometry_msgs::Quaternion TF2Wrapper::quatFromYaw(const double yaw)
{
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, yaw);

  return tf2::toMsg(quat);
}

double TF2Wrapper::yawFromQuat(const geometry_msgs::Quaternion& quat)
{
  double dummy;
  double yaw;

  tf2::Quaternion quat_tf;
  tf2::fromMsg(quat, quat_tf);
  tf2::Matrix3x3(quat_tf).getEulerYPR(yaw, dummy, dummy);

  return yaw;
}

tf2::Transform TF2Wrapper::poseToTransform(const geometry_msgs::Pose& pose)
{
  tf2::Vector3 v;
  tf2::fromMsg(pose.position, v);
  tf2::Quaternion r;
  tf2::fromMsg(pose.orientation, r);

  return tf2::Transform(r, v);
}

tf2_ros::Buffer& TF2Wrapper::buffer()
{
  static tf2_ros::Buffer tf_buffer;
  static tf2_ros::TransformListener tf_listener(tf_buffer);

  return tf_buffer;
}
