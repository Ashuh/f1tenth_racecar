#ifndef COLLISION_WARNING_SYSTEM_BICYCLE_MODEL_H
#define COLLISION_WARNING_SYSTEM_BICYCLE_MODEL_H

#include <string>
#include <vector>

#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>

#include "collision_warning_system/bicycle_state.h"

class BicycleModel
{
private:
  const double wheelbase_;

  BicycleState propagateState(const BicycleState state, const double delta_t);
  nav_msgs::Path bicycleStatesToPath(const std::vector<BicycleState> states, std::string frame_id);
  geometry_msgs::PoseStamped bicycleStateToPoseStamped(const BicycleState state, std::string frame_id);

public:
  explicit BicycleModel(const double wheelbase);

  nav_msgs::Path projectTrajectory(const geometry_msgs::Pose initial_pose, const std::string frame_id,
                                   const double velocity, const double steering_angle, const double delta_t,
                                   const double t_max);
};

#endif  // COLLISION_WARNING_SYSTEM_BICYCLE_MODEL_H
