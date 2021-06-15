#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include "local_planner/path.h"

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
