#ifndef LOCAL_PLANNER_VELOCITY_PROFILE_GENERATOR_H
#define LOCAL_PLANNER_VELOCITY_PROFILE_GENERATOR_H

#include "local_planner/path.h"
#include "local_planner/trajectory.h"

class VelocityProfileGenerator
{
private:
  const double max_lat_acc_;
  const double max_long_acc_;

public:
  VelocityProfileGenerator(const double max_lat_acc, const double max_long_acc);

  Trajectory generateVelocityProfile(Path& path, const double initial_velocity, const double goal_velocity);
};

#endif  // LOCAL_PLANNER_VELOCITY_PROFILE_GENERATOR_H
