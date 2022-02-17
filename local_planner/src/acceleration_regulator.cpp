#include "local_planner/acceleration_regulator.h"

#include <utility>
#include <vector>

AccelerationRegulator::AccelerationRegulator(const double max_speed, const double max_lat_acc, const double max_lon_acc,
                                             const double max_lon_dec)
{
  setMaxSpeed(max_speed);
  setMaxLateralAcceleration(max_lat_acc);
  setMaxLongitudinalAcceleration(max_lon_acc);
  setMaxLongitudinalDeceleration(max_lon_dec);
  initial_velocity_ = max_speed_;
}

std::vector<double> AccelerationRegulator::generateVelocityProfile(const Path& path) const
{
  std::vector<double> velocity_profile;

  for (int i = 0; i < path.size(); ++i)
  {
    double curvature_speed_limit = max_lat_acc_ / path.curvature(i);
    velocity_profile.push_back(std::min(max_speed_, curvature_speed_limit));
  }

  velocity_profile.front() = initial_velocity_;

  if (has_zero_final_velocity_)
  {
    velocity_profile.back() = 0;
  }

  do
  {
    std::vector<std::pair<int, int>> regions = identifyRegions(velocity_profile);

    for (const auto& region : regions)
    {
      if (velocity_profile.at(region.first) > velocity_profile.at(region.second))
      {
        // Decelerating region

        for (int i = region.second; i > region.first; --i)
        {
          double wp_distance = path.distance(i) - path.distance(i - 1);

          if (wp_distance == 0)
          {
            throw std::runtime_error("Distance is 0");
          }

          if (-getLonAcc(velocity_profile.at(i - 1), velocity_profile.at(i), wp_distance) > max_lon_dec_)
          {
            velocity_profile.at(i - 1) = getFinalVelocity(velocity_profile.at(i), max_lon_dec_, wp_distance);
          }
        }
      }
      else
      {
        // Accelerating region

        for (int i = region.first; i < region.second; ++i)
        {
          double wp_distance = path.distance(i + 1) - path.distance(i);

          if (wp_distance == 0)
          {
            throw std::runtime_error("Distance is 0");
          }

          double acc = getLonAcc(velocity_profile.at(i), velocity_profile.at(i + 1), wp_distance);

          if (getLonAcc(velocity_profile.at(i), velocity_profile.at(i + 1), wp_distance) > max_lon_acc_)
          {
            velocity_profile.at(i + 1) = getFinalVelocity(velocity_profile.at(i), max_lon_acc_, wp_distance);
          }
        }
      }
    }
  } while (!isValidProfile(path, velocity_profile));

  return velocity_profile;
}

void AccelerationRegulator::setInitialVelocity(const double velocity)
{
  initial_velocity_ = velocity;
}

void AccelerationRegulator::setMaxSpeed(const double speed)
{
  if (speed <= 0.0)
  {
    throw std::invalid_argument("Maximum speed must be be positive");
  }

  max_speed_ = speed;
}

void AccelerationRegulator::setMaxLateralAcceleration(const double acceleration)
{
  if (acceleration <= 0.0)
  {
    throw std::invalid_argument("Maximum lateral acceleration must be be positive");
  }

  max_lat_acc_ = acceleration;
}

void AccelerationRegulator::setMaxLongitudinalAcceleration(const double acceleration)
{
  if (acceleration <= 0.0)
  {
    throw std::invalid_argument("Maximum longitudinal acceleration must be be positive");
  }

  max_lon_acc_ = acceleration;
}

void AccelerationRegulator::setMaxLongitudinalDeceleration(const double deceleration)
{
  if (max_lon_dec_ <= 0.0)
  {
    throw std::invalid_argument("Maximum longitudinal deceleration must be be positive");
  }

  max_lon_dec_ = deceleration;
}

std::vector<std::pair<int, int>>
AccelerationRegulator::identifyRegions(const std::vector<double>& velocity_profile) const
{
  std::vector<int> region_boundaries;

  region_boundaries.push_back(0);

  for (int i = 1; i < velocity_profile.size() - 1; ++i)
  {
    if (velocity_profile.at(i - 1) > velocity_profile.at(i) && velocity_profile.at(i + 1) >= velocity_profile.at(i))
    {
      region_boundaries.push_back(i);
    }
    else if (velocity_profile.at(i - 1) < velocity_profile.at(i) &&
             velocity_profile.at(i + 1) <= velocity_profile.at(i))
    {
      region_boundaries.push_back(i);
    }
  }

  region_boundaries.push_back(velocity_profile.size() - 1);

  std::vector<std::pair<int, int>> regions;

  for (int i = 0; i < region_boundaries.size() - 1; i++)
  {
    regions.push_back({ region_boundaries.at(i), region_boundaries.at(i + 1) });
  }

  return regions;
}

void AccelerationRegulator::setZeroFinalVelocity(const bool has_zero_final_velocity)
{
  has_zero_final_velocity_ = has_zero_final_velocity;
}

bool AccelerationRegulator::isValidProfile(const Path& path, const std::vector<double>& velocity_profile) const
{
  for (int i = 0; i < path.size() - 1; ++i)
  {
    double lon_acc =
        getLonAcc(velocity_profile.at(i), velocity_profile.at(i + 1), path.distance(i + 1) - path.distance(i));

    if (lon_acc > max_lon_acc_ * 1.01 || -lon_acc > max_lon_dec_ * 1.01)
    {
      return false;
    }
  }

  return true;
}

double AccelerationRegulator::getLonAcc(const double v_i, const double v_f, const double s)
{
  return (pow(v_f, 2) - pow(v_i, 2)) / (2 * s);
}

double AccelerationRegulator::getFinalVelocity(const double v_i, const double a, const double s)
{
  return sqrt(pow(v_i, 2) + (2 * a * s));
}
