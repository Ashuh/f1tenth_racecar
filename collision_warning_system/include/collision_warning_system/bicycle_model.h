#ifndef COLLISION_WARNING_SYSTEM_BICYCLE_MODEL_H
#define COLLISION_WARNING_SYSTEM_BICYCLE_MODEL_H

#include <vector>
#include "collision_warning_system/bicycle_state.h"

namespace f1tenth_racecar
{
namespace safety
{
class BicycleModel
{
private:
  double wheelbase_;

  BicycleState propagateState(const BicycleState state, const double delta_t);

public:
  BicycleModel();
  explicit BicycleModel(const double wheel_base);

  std::vector<BicycleState> projectTrajectory(const BicycleState state, const double delta_t, const double steps);
};
}  // namespace safety
}  // namespace f1tenth_racecar

#endif  // COLLISION_WARNING_SYSTEM_BICYCLE_MODEL_H
