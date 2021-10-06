#ifndef LOCAL_PLANNER_CUBIC_VELOCITY_TIME_PROFILE_H
#define LOCAL_PLANNER_CUBIC_VELOCITY_TIME_PROFILE_H

#include <Eigen/Dense>

class CubicVelocityTimeProfile
{
public:
  CubicVelocityTimeProfile(const double v_i, const double v_f, const double s_f);

  Eigen::Vector4d getCoefficients() const;

  double getEndTime() const;

  double getDisplacementAtTime(const double t) const;

  double getVelocityAtTime(const double t) const;

  double getAccelerationAtTime(const double t) const;

  double getFinalDisplacement() const;

  double getFinalVelocity() const;

  double getFinalAcceleration() const;

  double getTimeAtDisplacement(const double s) const;

private:
  Eigen::Vector4d a_;  // coefficients
  double t_f_;         // end time

  Eigen::VectorXd stripZeroCoefficients(const Eigen::Matrix<double, 5, 1>& coeff) const;
};

#endif  // LOCAL_PLANNER_CUBIC_VELOCITY_TIME_PROFILE_H
