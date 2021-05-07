#include "collision_warning_system/bicycle_state.h"

namespace f1tenth_racecar
{
namespace safety
{
BicycleState::BicycleState(double x, double y, double v, double yaw, double steering_angle)
{
  x_ = x;
  y_ = y;
  v_ = v;
  yaw_ = yaw;
  steering_angle_ = steering_angle;
}

double BicycleState::x() const
{
  return x_;
}

double BicycleState::y() const
{
  return y_;
}

double BicycleState::v() const
{
  return v_;
}

double BicycleState::yaw() const
{
  return yaw_;
}

double BicycleState::steering_angle() const
{
  return steering_angle_;
}
}  // namespace safety
}  // namespace f1tenth_racecar
