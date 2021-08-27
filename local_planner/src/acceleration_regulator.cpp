#include <utility>
#include <vector>

#include "local_planner/acceleration_regulator.h"

AccelerationRegulator::Constraints::Constraints(const double max_speed, const double max_lat_acc,
                                                const double max_lon_acc, const double max_lon_dec)
{
  if (max_speed <= 0.0 || max_lat_acc <= 0.0 || max_lon_acc <= 0.0 || max_lon_dec <= 0.0)
  {
    throw std::invalid_argument("Invalid constraints specified");
  }

  max_speed_ = max_speed;
  max_lat_acc_ = max_lat_acc;
  max_lon_acc_ = max_lon_acc;
  max_lon_dec_ = max_lon_dec;
}

AccelerationRegulator::AccelerationRegulator(const Constraints& constraints) : constraints_(constraints)
{
}

std::vector<double> AccelerationRegulator::generateVelocityProfile(const Path& path)
{
  std::vector<double> velocity_profile;

  for (int i = 0; i < path.size(); ++i)
  {
    double curvature_speed_limit = constraints_.max_lat_acc_ / path.curvature(i);
    velocity_profile.push_back(std::min(constraints_.max_speed_, curvature_speed_limit));
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

          if (-getLonAcc(velocity_profile.at(i - 1), velocity_profile.at(i), wp_distance) > constraints_.max_lon_dec_)
          {
            velocity_profile.at(i - 1) =
                getFinalVelocity(velocity_profile.at(i), constraints_.max_lon_dec_, wp_distance);
          }
        }
      }
      else
      {
        // Accelerating region

        for (int i = region.first; i < region.second; ++i)
        {
          double wp_distance = path.distance(i + 1) - path.distance(i);

          double acc = getLonAcc(velocity_profile.at(i), velocity_profile.at(i + 1), wp_distance);

          if (getLonAcc(velocity_profile.at(i), velocity_profile.at(i + 1), wp_distance) > constraints_.max_lon_acc_)
          {
            velocity_profile.at(i + 1) =
                getFinalVelocity(velocity_profile.at(i), constraints_.max_lon_acc_, wp_distance);
          }
        }
      }
    }
  } while (!isValidProfile(path, velocity_profile));

  return velocity_profile;
}

void AccelerationRegulator::setConstraints(const Constraints& constraints)
{
  constraints_ = constraints;
}

std::vector<std::pair<int, int>> AccelerationRegulator::identifyRegions(const std::vector<double>& velocity_profile)
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

bool AccelerationRegulator::isValidProfile(const Path& path, const std::vector<double>& velocity_profile)
{
  for (int i = 0; i < path.size() - 1; ++i)
  {
    double lon_acc =
        getLonAcc(velocity_profile.at(i), velocity_profile.at(i + 1), path.distance(i + 1) - path.distance(i));

    if (lon_acc > constraints_.max_lon_acc_ * 1.01 || -lon_acc > constraints_.max_lon_dec_ * 1.01)
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
