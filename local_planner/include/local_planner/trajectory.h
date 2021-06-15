#ifndef LOCAL_PLANNER_TRAJECTORY_H
#define LOCAL_PLANNER_TRAJECTORY_H

#include <vector>

#include "local_planner/path.h"

class Trajectory : public Path
{
private:
  std::vector<double> velocity_;

public:
  Trajectory(const Path& path, const std::vector<double>& velocity);

  double velocity(size_t n) const;
};

#endif  // LOCAL_PLANNER_TRAJECTORY_H
