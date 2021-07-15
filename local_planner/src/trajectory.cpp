#include <vector>

#include "local_planner/path.h"
#include "local_planner/trajectory.h"

Trajectory::Trajectory()
{
}

Trajectory::Trajectory(const Path& path, const std::vector<double>& velocity) : Path(path)
{
  velocity_ = velocity;
}

double Trajectory::velocity(size_t n) const
{
  return velocity_.at(n);
}
