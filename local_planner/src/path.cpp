#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "local_planner/path.h"

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
