#include <limits>
#include <vector>

#include <unsupported/Eigen/Polynomials>

#include "local_planner/cubic_velocity_time_profile.h"

CubicVelocityTimeProfile::CubicVelocityTimeProfile(const double v_i, const double v_f, const double s_f)
{
  if (s_f <= 0.0)
  {
    throw std::invalid_argument("Displacement must be greater than 0");
  }
  if (v_i < 0.0 || v_f < 0.0)
  {
    throw std::invalid_argument("Initial and final velocity must be greater than or equal to 0");
  }
  if (v_i == 0.0 && v_f == 0.0)
  {
    throw std::invalid_argument("Initial and final velocity cannot both be equal to 0");
  }

  a_(0) = v_i;
  a_(1) = 0.0;
  a_(2) = -(3 * (-pow(v_f, 3) - pow(v_f, 2) * v_i + v_f * pow(v_i, 2) + pow(v_i, 3))) / (4 * pow(s_f, 2));
  a_(3) = (-pow(v_f, 4) - 2 * pow(v_f, 3) * v_i + 2 * v_f * pow(v_i, 3) + pow(v_i, 4)) / (4 * pow(s_f, 3));
  t_f_ = (2 * s_f) / (v_f + v_i);
}

Eigen::Vector4d CubicVelocityTimeProfile::getCoefficients() const
{
  return a_;
}

double CubicVelocityTimeProfile::getEndTime() const
{
  return t_f_;
}

double CubicVelocityTimeProfile::getDisplacementAtTime(const double t) const
{
  return a_(0) * t + 1.0 / 2.0 * a_(1) * pow(t, 2) + 1.0 / 3.0 * a_(2) * pow(t, 3) + 1.0 / 4.0 * a_(3) * pow(t, 4);
}

double CubicVelocityTimeProfile::getVelocityAtTime(const double t) const
{
  return a_(0) + a_(1) * t + a_(2) * pow(t, 2) + a_(3) * pow(t, 3);
}

double CubicVelocityTimeProfile::getAccelerationAtTime(const double t) const
{
  return a_(1) + 2 * a_(2) * t + 3 * a_(3) * pow(t, 2);
}

double CubicVelocityTimeProfile::getFinalDisplacement() const
{
  return getDisplacementAtTime(t_f_);
}

double CubicVelocityTimeProfile::getFinalVelocity() const
{
  return getVelocityAtTime(t_f_);
}

double CubicVelocityTimeProfile::getFinalAcceleration() const
{
  return getAccelerationAtTime(t_f_);
}

double CubicVelocityTimeProfile::getTimeAtDisplacement(const double s) const
{
  if (s == 0.0)
  {
    return 0.0;
  }

  double s_f = getFinalDisplacement();

  if (std::abs(s - s_f) <= s_f * 0.001)
  {
    return t_f_;
  }

  if (s < 0 || s > s_f)
  {
    throw std::invalid_argument("Invalid displacement");
  }

  Eigen::PolynomialSolver<double, Eigen::Dynamic> solver;
  Eigen::Matrix<double, 5, 1> coeff;
  coeff << -s, a_(0), 1.0 / 2.0 * a_(1), 1.0 / 3.0 * a_(2), 1.0 / 4.0 * a_(3);
  Eigen::VectorXd stripped_coeff = stripZeroCoefficients(coeff);
  solver.compute(stripped_coeff);
  std::vector<double> real_roots;
  solver.realRoots(real_roots, std::numeric_limits<double>::epsilon());

  for (auto& t : real_roots)
  {
    if (t >= 0 && t <= t_f_ * 1.001)
    {
      return t;
    }
  }

  std::ostringstream o;
  o << "Unable to solve [" << coeff(0) << " " << coeff(1) << " t " << coeff(2) << " t^2 " << coeff(3) << " t^3 "
    << coeff(4) << " t^4]"
    << " for time at displacement " << s;

  throw std::runtime_error(o.str());
}

Eigen::VectorXd CubicVelocityTimeProfile::stripZeroCoefficients(const Eigen::Matrix<double, 5, 1>& coeff) const
{
  int degree = 4;

  while (degree > 0)
  {
    if (coeff(degree) == 0.0)
    {
      degree--;
    }
    else
    {
      break;
    }
  }

  Eigen::VectorXd stripped_coeff(degree + 1);

  for (int i = 0; i <= degree; ++i)
  {
    stripped_coeff[i] = coeff(i);
  }

  return stripped_coeff;
}
