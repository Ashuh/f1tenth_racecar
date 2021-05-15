#include <cmath>
#include <vector>
#include <assert.h>
#include "collision_warning_system/bicycle_model.h"

namespace f1tenth_racecar
{
namespace safety
{
BicycleModel::BicycleModel()
{
  wheelbase_ = -1;
}

BicycleModel::BicycleModel(const double wheelbase)
{
  wheelbase_ = wheelbase;
}

std::vector<BicycleState> BicycleModel::projectTrajectory(const BicycleState state, const double delta_t,
                                                          const double steps)
{
  assert(wheelbase_ > 0);
  std::vector<BicycleState> projected_trajectory;
  projected_trajectory.push_back(state);
  BicycleState next_state = state;

  for (int i = 0; i < steps; i++)
  {
    next_state = propagateState(next_state, delta_t);
    projected_trajectory.push_back(next_state);
  }

  return projected_trajectory;
}

BicycleState BicycleModel::propagateState(const BicycleState state, const double delta_t)
{
  double v_x = state.v() * cos(state.yaw());
  double v_y = state.v() * sin(state.yaw());
  double yaw_rate = state.v() * tan(state.steering_angle()) / wheelbase_;
  double next_x = state.x() + v_x * delta_t;
  double next_y = state.y() + v_y * delta_t;
  double next_yaw = state.yaw() + yaw_rate * delta_t;
  return BicycleState(next_x, next_y, state.v(), next_yaw, state.steering_angle());
}
}  // namespace safety
}  // namespace f1tenth_racecar
