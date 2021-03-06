#ifndef COLLISION_WARNING_SYSTEM_BICYCLE_STATE_H
#define COLLISION_WARNING_SYSTEM_BICYCLE_STATE_H

class BicycleState
{
private:
  double x_;
  double y_;
  double v_;
  double yaw_;
  double steering_angle_;

public:
  BicycleState(double x, double y, double v, double yaw, double steering_angle);

  double x() const;
  double y() const;
  double v() const;
  double yaw() const;
  double steering_angle() const;
};

#endif  // COLLISION_WARNING_SYSTEM_BICYCLE_STATE_H
