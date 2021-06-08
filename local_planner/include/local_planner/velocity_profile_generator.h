#ifndef LOCAL_PLANNER_VELOCITY_PROFILE_GENERATOR_H
#define LOCAL_PLANNER_VELOCITY_PROFILE_GENERATOR_H

#include "local_planner/cubic_spiral_path.h"

class VelocityProfileGenerator
{
private:
  const double max_lat_acc_;
  const double max_long_acc_;

public:
  VelocityProfileGenerator(const double max_lat_acc, const double max_long_acc);

  void generateVelocityProfile(CubicSpiralPath& path, const double initial_velocity, const double goal_velocity);
};

#endif  // LOCAL_PLANNER_VELOCITY_PROFILE_GENERATOR_H
