#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "local_planner/velocity_profile_generator.h"
#include "local_planner/path.h"
#include "local_planner/trajectory.h"

VelocityProfileGenerator::VelocityProfileGenerator(const double max_lat_acc, const double max_long_acc)
  : max_lat_acc_(max_lat_acc), max_long_acc_(max_lat_acc)
{
}

Trajectory VelocityProfileGenerator::generateVelocityProfile(Path& path, const double initial_velocity,
                                                             const double goal_velocity)
{
  double max_curvature = -std::numeric_limits<double>::max();
  // int max_curvature_id = -1;

  for (int i = 0; i < path.size(); ++i)
  {
    double curvature = path.curvature(i);

    if (curvature > max_curvature)
    {
      max_curvature = curvature;
      // max_curvature_id = i;
    }
  }

  double v_max_curvature = sqrt(max_lat_acc_ / max_curvature);
  double v_max = std::min(v_max_curvature, goal_velocity);
  std::vector<double> velocity;

  for (int i = 0; i < path.size(); ++i)
  {
    velocity.push_back(v_max);
  }

  return Trajectory(path, velocity);
}
