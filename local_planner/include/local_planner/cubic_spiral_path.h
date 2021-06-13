#ifndef LOCAL_PLANNER_CUBIC_SPIRAL_PATH_H
#define LOCAL_PLANNER_CUBIC_SPIRAL_PATH_H

#include <string>
#include <vector>

#include "local_planner/waypoint.h"

class CubicSpiralPath
{
private:
  std::vector<Waypoint> waypoints_;

  double cost_;

public:
  CubicSpiralPath();

  void addWaypoint(const Waypoint& waypoint);

  void setCost(const double cost);

  double getCost() const;

  void setVelocity(const size_t n, const double velocity);

  size_t size() const;

  Waypoint at(size_t n) const;

  std::string toString(uint precision = 3) const;
};

#endif  // LOCAL_PLANNER_CUBIC_SPIRAL_PATH_H
