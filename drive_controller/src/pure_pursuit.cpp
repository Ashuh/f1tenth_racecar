#include <limits>

#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "drive_controller/pure_pursuit.h"

namespace f1tenth_racecar
{
namespace control
{
PurePursuit::PurePursuit(double look_ahead_dist, double gain)
  : tf_listener_(tf_buffer_), look_ahead_dist_(look_ahead_dist), gain_(gain)
{
}

double PurePursuit::getDist(const geometry_msgs::Point point_1, const geometry_msgs::Point point_2)
{
  double d_x = point_1.x - point_2.x;
  double d_y = point_1.y - point_2.y;

  return sqrt(pow(d_x, 2) + pow(d_y, 2));
}

bool PurePursuit::isPoseAhead(const nav_msgs::Odometry odom, const geometry_msgs::PoseStamped pose)
{
  geometry_msgs::TransformStamped transform =
      tf_buffer_.lookupTransform(odom.child_frame_id, pose.header.frame_id, ros::Time(0));
  geometry_msgs::Point point_transformed;
  tf2::doTransform(pose.pose.position, point_transformed, transform);

  return point_transformed.x > 0;
}

geometry_msgs::PointStamped PurePursuit::findLookAheadPoint(nav_msgs::Odometry odom, const nav_msgs::Path path)
{
  look_ahead_point_dist_ = -std::numeric_limits<double>::max();
  geometry_msgs::PointStamped look_ahead_point;

  for (int i = 0; i < path.poses.size(); ++i)
  {
    double dist = getDist(path.poses.at(i).pose.position, odom.pose.pose.position);

    if (dist < look_ahead_dist_ && dist > look_ahead_point_dist_ && isPoseAhead(odom, path.poses.at(i)))
    {
      look_ahead_point_dist_ = dist;
      look_ahead_point.header.frame_id = path.header.frame_id;
      look_ahead_point.point = path.poses.at(i).pose.position;
    }
  }

  if (look_ahead_point_dist_ < 0)
  {
    throw std::runtime_error("No valid look ahead point found in path");
  }

  return look_ahead_point;
}

double PurePursuit::calculateSteeringAngle(const nav_msgs::Odometry odom, const nav_msgs::Path path)
{
  if (path.poses.empty())
  {
    throw std::invalid_argument("Path is empty");
  }
  if (odom.header.frame_id != path.header.frame_id)
  {
    throw std::invalid_argument("Vehicle pose and path are in different frames");
  }

  look_ahead_point_ = findLookAheadPoint(odom, path);

  geometry_msgs::TransformStamped transform =
      tf_buffer_.lookupTransform(odom.child_frame_id, look_ahead_point_.header.frame_id, ros::Time(0));
  geometry_msgs::PointStamped look_ahead_point_transformed;
  tf2::doTransform(look_ahead_point_, look_ahead_point_transformed, transform);

  arc_radius_ = pow(look_ahead_point_dist_, 2) / (2 * abs(look_ahead_point_transformed.point.y));
  double d = arc_radius_ - abs(look_ahead_point_transformed.point.y);

  arc_center_.header.frame_id = odom.child_frame_id;
  arc_center_.point.x = 0.0;
  arc_center_.point.y = (look_ahead_point_transformed.point.y > 0) ? look_ahead_point_transformed.point.y + d :
                                                                     look_ahead_point_transformed.point.y - d;

  return ((look_ahead_point_transformed.point.y > 0) ? 1 : -1) * gain_ / arc_radius_;
}

void PurePursuit::getIntermediateResults(geometry_msgs::PointStamped& look_ahead_point, double& look_ahead_point_dist,
                                         geometry_msgs::PointStamped& arc_center, double& arc_radius)
{
  look_ahead_point = look_ahead_point_;
  look_ahead_point_dist = look_ahead_point_dist_;
  arc_center = arc_center_;
  arc_radius = arc_radius_;
}
}  // namespace control
}  // namespace f1tenth_racecar
