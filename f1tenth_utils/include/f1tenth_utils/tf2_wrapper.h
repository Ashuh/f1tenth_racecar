#ifndef F1TENTH_UTILS_TF2_WRAPPER_H
#define F1TENTH_UTILS_TF2_WRAPPER_H

#include <string>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

class TF2Wrapper
{
public:
  static geometry_msgs::TransformStamped lookupTransform(const std::string& target_frame,
                                                         const std::string& source_frame,
                                                         const ros::Time& time = ros::Time(0));

  template <typename T>
  static T doTransform(const T& data, const std::string& target_frame, const std::string& source_frame)
  {
    T transformed;
    tf2::doTransform(data, transformed, lookupTransform(target_frame, source_frame));

    return transformed;
  }

  template <typename T>
  static T doTransform(const T& data, const std::string& target_frame)
  {
    T transformed;
    tf2::doTransform(data, transformed, lookupTransform(target_frame, data.header.frame_id));

    return transformed;
  }

  template <typename T>
  static T doTransform(const T& data, const geometry_msgs::Pose& target_pose)
  {
    T transformed;
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.transform = tf2::toMsg(poseToTransform(target_pose).inverse());
    tf2::doTransform(data, transformed, transform_stamped);

    return transformed;
  }

  static geometry_msgs::Quaternion quatFromYaw(const double yaw);

  static double yawFromQuat(const geometry_msgs::Quaternion& quat);

private:
  static tf2::Transform poseToTransform(const geometry_msgs::Pose& pose);

  static tf2_ros::Buffer& buffer();
};

#endif  // F1TENTH_UTILS_TF2_WRAPPER_H
