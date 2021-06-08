#include <algorithm>
#include <cmath>
#include <limits>

#include "local_planner/velocity_profile_generator.h"
#include "local_planner/cubic_spiral_path.h"
#include "local_planner/waypoint.h"

VelocityProfileGenerator::VelocityProfileGenerator(const double max_lat_acc, const double max_long_acc)
  : max_lat_acc_(max_lat_acc), max_long_acc_(max_lat_acc)
{
}

void VelocityProfileGenerator::generateVelocityProfile(CubicSpiralPath& path, const double initial_velocity,
                                                       const double goal_velocity)
{
  double max_curvature = -std::numeric_limits<double>::max();
  int max_curvature_id = -1;

  for (int i = 0; i < path.size(); ++i)
  {
    double curvature = path.at(i).curvature_;

    if (curvature > max_curvature)
    {
      max_curvature = curvature;
      max_curvature_id = i;
    }
  }

  double v_max_curvature = sqrt(max_lat_acc_ / max_curvature);
  double v_max = std::min(v_max_curvature, goal_velocity);
  // double acc = (pow(v_max, 2) - pow(initial_velocity, 2)) / (2 * path.at(max_curvature_id).distance_);

  for (int i = 0; i < path.size(); ++i)
  {
    path.setVelocity(i, v_max);
  }
}
