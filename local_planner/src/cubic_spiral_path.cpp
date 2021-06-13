#include <iomanip>
#include <string>
#include <sstream>
#include <vector>

#include "local_planner/cubic_spiral_path.h"

CubicSpiralPath::CubicSpiralPath()
{
}

void CubicSpiralPath::addWaypoint(const Waypoint& waypoint)
{
  waypoints_.push_back(waypoint);
}

void CubicSpiralPath::setCost(const double cost)
{
  cost_ = cost;
}

double CubicSpiralPath::getCost() const
{
  return cost_;
}

void CubicSpiralPath::setVelocity(const size_t n, const double velocity)
{
  waypoints_.at(n).velocity_ = velocity;
}

size_t CubicSpiralPath::size() const
{
  return waypoints_.size();
}

Waypoint CubicSpiralPath::at(size_t n) const
{
  return waypoints_.at(n);
}

std::string CubicSpiralPath::toString(uint precision) const
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(precision);
  oss << std::endl;

  oss << "s  : ";
  for (int i = 0; i < size(); ++i)
  {
    oss << waypoints_.at(i).distance_ << " ";
  }
  oss << std::endl;

  oss << "x  : ";
  for (int i = 0; i < size(); ++i)
  {
    oss << waypoints_.at(i).x_ << " ";
  }
  oss << std::endl;

  oss << "y  : ";
  for (int i = 0; i < size(); ++i)
  {
    oss << waypoints_.at(i).y_ << " ";
  }
  oss << std::endl;

  oss << "yaw: ";
  for (int i = 0; i < size(); ++i)
  {
    oss << waypoints_.at(i).yaw_ << " ";
  }
  oss << std::endl;

  oss << "v  : ";
  for (int i = 0; i < size(); ++i)
  {
    oss << waypoints_.at(i).velocity_ << " ";
  }
  oss << std::endl;

  oss << "k  : ";
  for (int i = 0; i < size(); ++i)
  {
    oss << waypoints_.at(i).curvature_ << " ";
  }
  oss << std::endl;

  return oss.str();
}
