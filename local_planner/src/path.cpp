#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>

#include "local_planner/path.h"
#include "f1tenth_utils/tf2_wrapper.h"

Path::Path()
{
}

Path::Path(const std::string& frame_id, const std::vector<double>& distance, const std::vector<double>& x,
           const std::vector<double>& y, const std::vector<double>& yaw, const std::vector<double>& curvature)
{
  frame_id_ = frame_id;
  distance_ = distance;
  x_ = x;
  y_ = y;
  yaw_ = yaw;
  curvature_ = curvature;
  size_ = distance.size();
}

Path Path::trim(const size_t begin, const size_t end) const
{
  std::vector<double> dist = trimVector(distance_, begin, end);
  std::vector<double> x = trimVector(x_, begin, end);
  std::vector<double> y = trimVector(y_, begin, end);
  std::vector<double> yaw = trimVector(yaw_, begin, end);
  std::vector<double> curvature = trimVector(curvature_, begin, end);

  return Path(frame_id_, dist, x, y, yaw, curvature);
}

std::string Path::getFrameId() const
{
  return frame_id_;
}

size_t Path::size() const
{
  return size_;
}

double Path::distance(size_t n) const
{
  return distance_.at(n);
}

double Path::x(size_t n) const
{
  return x_.at(n);
}

double Path::y(size_t n) const
{
  return y_.at(n);
}

double Path::yaw(size_t n) const
{
  return yaw_.at(n);
}

double Path::curvature(size_t n) const
{
  return curvature_.at(n);
}

Path::IteratorPair Path::distanceIt() const
{
  return IteratorPair(distance_.begin(), distance_.end());
}

Path::IteratorPair Path::xIt() const
{
  return IteratorPair(x_.begin(), x_.end());
}

Path::IteratorPair Path::yIt() const
{
  return IteratorPair(y_.begin(), y_.end());
}

Path::IteratorPair Path::yawIt() const
{
  return IteratorPair(yaw_.begin(), yaw_.end());
}

Path::IteratorPair Path::curavatureIt() const
{
  return IteratorPair(curvature_.begin(), curvature_.end());
}

geometry_msgs::Point Path::point(size_t n) const
{
  geometry_msgs::Point point;

  point.x = x_.at(n);
  point.y = y_.at(n);

  return point;
}

geometry_msgs::Pose Path::pose(size_t n) const
{
  geometry_msgs::Pose pose;
  pose.position = point(n);

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw_.at(n));
  pose.orientation = tf2::toMsg(q);

  return pose;
}

geometry_msgs::PoseStamped Path::poseStamped(size_t n) const
{
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.pose = pose(n);
  pose_stamped.header.frame_id = frame_id_;

  return pose_stamped;
}

Path Path::transform(const std::string target_frame) const
{
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> yaw;

  for (int i = 0; i < size_; ++i)
  {
    geometry_msgs::PoseStamped pose_transformed =
        TF2Wrapper::doTransform<geometry_msgs::PoseStamped>(poseStamped(i), target_frame);
    x.push_back(pose_transformed.pose.position.x);
    y.push_back(pose_transformed.pose.position.y);
    yaw.push_back(TF2Wrapper::yawFromQuat(pose_transformed.pose.orientation));
  }

  return Path(target_frame, distance_, x, y, yaw, curvature_);
}

visualization_msgs::Marker Path::generatePathMarker(const int marker_id, const std::string& ns, const double scale,
                                                    const double r, const double g, const double b,
                                                    const double a) const
{
  visualization_msgs::Marker path_marker;
  path_marker.action = visualization_msgs::Marker::ADD;
  path_marker.type = visualization_msgs::Marker::LINE_STRIP;
  path_marker.ns = ns;
  path_marker.id = marker_id;
  path_marker.lifetime = ros::Duration(0.1);
  path_marker.header.frame_id = frame_id_;
  path_marker.pose.orientation.w = 1.0;
  path_marker.scale.x = scale;
  path_marker.color.r = r;
  path_marker.color.g = g;
  path_marker.color.b = b;
  path_marker.color.a = a;

  for (int i = 0; i < size_; ++i)
  {
    path_marker.points.push_back(point(i));
  }

  return path_marker;
}

std::vector<double> Path::trimVector(const std::vector<double>& vec, const size_t begin, const size_t end)
{
  return std::vector<double>(vec.begin() + begin, vec.begin() + end + 1);
}
