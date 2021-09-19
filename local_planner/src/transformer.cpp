#include "local_planner/transformer.h"

geometry_msgs::Pose Transformer::transform(const geometry_msgs::Pose& pose, const std::string& target_frame,
                                           const std::string& source_frame)
{
  geometry_msgs::Pose pose_transformed;
  tf2::doTransform(pose, pose_transformed, buffer().lookupTransform(target_frame, source_frame, ros::Time(0)));

  return pose_transformed;
}

geometry_msgs::PoseStamped Transformer::transform(const geometry_msgs::PoseStamped& pose,
                                                  const std::string& target_frame)
{
  geometry_msgs::PoseStamped pose_transformed;
  tf2::doTransform(pose, pose_transformed, buffer().lookupTransform(target_frame, pose.header.frame_id, ros::Time(0)));

  return pose_transformed;
}

Path Transformer::transform(const Path& path, const std::string& target_frame)
{
  Path transformed;

  std::vector<double> distance;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> yaw;
  std::vector<double> curvature;

  for (int i = 0; i < path.size(); ++i)
  {
    geometry_msgs::PoseStamped pose_transformed = transform(path.poseStamped(i), target_frame);
    x.push_back(pose_transformed.pose.position.x);
    y.push_back(pose_transformed.pose.position.y);
    yaw.push_back(yawFromQuat(pose_transformed.pose.orientation));
    distance.push_back(path.distance(i));
    curvature.push_back(path.curvature(i));
  }

  return Path(target_frame, distance, x, y, yaw, curvature);
}

geometry_msgs::Point Transformer::transform(const geometry_msgs::Pose& ref_pose,
                                            const geometry_msgs::Point& target_point)
{
  tf2::Vector3 v;
  tf2::fromMsg(target_point, v);
  tf2::Vector3 result = poseToTransform(ref_pose).inverse() * v;

  geometry_msgs::Point point_transformed;
  tf2::toMsg(result, point_transformed);

  return point_transformed;
}

geometry_msgs::Pose Transformer::transform(const geometry_msgs::Pose& ref_pose, const geometry_msgs::Pose& target_pose)
{
  tf2::Transform result = poseToTransform(ref_pose).inverse() * poseToTransform(target_pose);
  geometry_msgs::Pose pose_transformed;
  tf2::toMsg(result, pose_transformed);

  return pose_transformed;
}
geometry_msgs::Quaternion Transformer::quatFromYaw(const double yaw)
{
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, yaw);

  return tf2::toMsg(quat);
}

double Transformer::yawFromQuat(const geometry_msgs::Quaternion quat)
{
  double dummy;
  double yaw;

  tf2::Quaternion quat_tf;
  tf2::fromMsg(quat, quat_tf);
  tf2::Matrix3x3(quat_tf).getEulerYPR(yaw, dummy, dummy);

  return yaw;
}

tf2::Transform Transformer::poseToTransform(const geometry_msgs::Pose& pose)
{
  tf2::Vector3 v;
  tf2::fromMsg(pose.position, v);
  tf2::Quaternion r;
  tf2::fromMsg(pose.orientation, r);

  return tf2::Transform(r, v);
}

tf2_ros::Buffer& Transformer::buffer()
{
  static tf2_ros::Buffer tf_buffer;
  static tf2_ros::TransformListener tf_listener(tf_buffer);

  return tf_buffer;
}
