#include <algorithm>
#include <utility>
#include <vector>

#include <geometry_msgs/Point.h>
#include <unsupported/Eigen/Splines>

#include "local_planner/path.h"
#include "local_planner/trajectory.h"
#include "local_planner/reference_trajectory_generator.h"

ReferenceTrajectoryGenerator::ReferenceTrajectoryGenerator(const double speed_limit, const double max_lat_acc,
                                                           const double max_lon_acc, const double max_lon_dec)
{
  setSpeedLimit(speed_limit);
  setMaxLatAcc(max_lat_acc);
  setMaxLonAcc(max_lon_acc);
  setMaxLonDec(max_lon_dec);
}

Trajectory
ReferenceTrajectoryGenerator::generateReferenceTrajectory(const std::vector<geometry_msgs::Point>& lattice_vertices)
{
  std::vector<geometry_msgs::Point> ref_path_points = cubicSplineInterpolate(lattice_vertices);

  Path reference_path = generateReferencePath(lattice_vertices);
  std::vector<double> velocity_profile = generateVelocityProfile(reference_path);

  return Trajectory(reference_path, velocity_profile);
}

Path ReferenceTrajectoryGenerator::generateReferencePath(const std::vector<geometry_msgs::Point>& lattice_vertices)
{
  std::vector<geometry_msgs::Point> ref_path_points = cubicSplineInterpolate(lattice_vertices);

  std::vector<double> distance_vec;
  std::vector<double> x_vec;
  std::vector<double> y_vec;
  std::vector<double> yaw_vec;
  std::vector<double> curvature_vec;

  for (int i = 0; i < ref_path_points.size(); ++i)
  {
    distance_vec.push_back(
        (i == 0) ? 0.0 : distance_vec.at(i - 1) + distance(ref_path_points.at(i), ref_path_points.at(i - 1)));

    if (i == ref_path_points.size() - 1)
    {
      yaw_vec.push_back(yaw_vec.at(i - 1));
    }
    else
    {
      double dx = ref_path_points.at(i + 1).x - ref_path_points.at(i).x;
      double dy = ref_path_points.at(i + 1).y - ref_path_points.at(i).y;
      double yaw = atan2(dy, dx);
      yaw_vec.push_back(yaw);
    }

    x_vec.push_back(ref_path_points.at(i).x);
    y_vec.push_back(ref_path_points.at(i).y);

    if (i == 0)
    {
      curvature_vec.push_back(
          mengerCurvature(ref_path_points.at(i), ref_path_points.at(i + 1), ref_path_points.at(i + 2)));
    }
    else if (i == ref_path_points.size() - 1)
    {
      curvature_vec.push_back(
          mengerCurvature(ref_path_points.at(i - 2), ref_path_points.at(i - 1), ref_path_points.at(i)));
    }
    else
    {
      curvature_vec.push_back(
          mengerCurvature(ref_path_points.at(i - 1), ref_path_points.at(i), ref_path_points.at(i + 1)));
    }
  }

  Path reference_path("map", distance_vec, x_vec, y_vec, yaw_vec, curvature_vec);

  return reference_path;
}

std::vector<double> ReferenceTrajectoryGenerator::generateVelocityProfile(const Path& path)
{
  std::vector<double> velocity_profile;

  for (int i = 0; i < path.size(); ++i)
  {
    double curvature_speed_limit = max_lat_acc_ / path.curvature(i);
    velocity_profile.push_back(std::min(speed_limit_, curvature_speed_limit));
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

std::vector<std::pair<int, int>>
ReferenceTrajectoryGenerator::identifyRegions(const std::vector<double>& velocity_profile)
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

bool ReferenceTrajectoryGenerator::isValidProfile(const Path& path, const std::vector<double>& velocity_profile)
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

std::vector<geometry_msgs::Point>
ReferenceTrajectoryGenerator::cubicSplineInterpolate(const std::vector<geometry_msgs::Point>& path)
{
  Eigen::VectorXd xvals(path.size());
  Eigen::VectorXd yvals(path.size());

  for (int i = 0; i < path.size(); i++)
  {
    geometry_msgs::Point point = path.at(i);
    xvals(i) = point.x;
    yvals(i) = point.y;
  }

  Eigen::Spline<double, 2>::ControlPointVectorType ref_path_points(2, path.size());
  ref_path_points.row(0) = xvals;
  ref_path_points.row(1) = yvals;
  const Eigen::Spline<double, 2> spline =
      Eigen::SplineFitting<Eigen::Spline<double, 2>>::Interpolate(ref_path_points, 3);

  std::vector<geometry_msgs::Point> spline_ref_path_points;

  for (double u = 0; u <= 1; u += 0.025)
  {
    Eigen::Spline<double, 2>::PointType p = spline(u);

    geometry_msgs::Point point;
    point.x = p(0, 0);
    point.y = p(1, 0);

    spline_ref_path_points.push_back(point);
  }

  std::reverse(spline_ref_path_points.begin(), spline_ref_path_points.end());

  return spline_ref_path_points;
}

double ReferenceTrajectoryGenerator::getLonAcc(const double v_i, const double v_f, const double s)
{
  return (pow(v_f, 2) - pow(v_i, 2)) / (2 * s);
}

double ReferenceTrajectoryGenerator::getFinalVelocity(const double v_i, const double a, const double s)
{
  return sqrt(pow(v_i, 2) + (2 * a * s));
}

double ReferenceTrajectoryGenerator::distance(const geometry_msgs::Point& a, const geometry_msgs::Point& b)
{
  double d_x = a.x - b.x;
  double d_y = a.y - b.y;
  return sqrt(pow(d_x, 2) + pow(d_y, 2));
}

double ReferenceTrajectoryGenerator::triangleArea(const double length_ab, const double length_bc,
                                                  const double length_ca)
{
  return 0.25 * sqrt((length_ab + length_bc + length_ca) * (-length_ab + length_bc + length_ca) *
                     (length_ab - length_bc + length_ca) * (length_ab + length_bc - length_ca));
}

double ReferenceTrajectoryGenerator::mengerCurvature(const geometry_msgs::Point& a, const geometry_msgs::Point& b,
                                                     const geometry_msgs::Point& c)
{
  double length_ab = distance(a, b);
  double length_bc = distance(b, c);
  double length_ca = distance(c, a);
  double area = triangleArea(length_ab, length_bc, length_ca);

  return (4 * area) / (length_ab * length_bc * length_ca);
}

void ReferenceTrajectoryGenerator::setSpeedLimit(const double speed_limit)
{
  if (speed_limit > 0.0)
  {
    speed_limit_ = speed_limit;
  }
  else
  {
    throw std::invalid_argument("Speed limit must be positive");
  }
}

void ReferenceTrajectoryGenerator::setMaxLatAcc(const double max_lat_acc)
{
  if (max_lat_acc > 0.0)
  {
    max_lat_acc_ = max_lat_acc;
  }
  else
  {
    throw std::invalid_argument("Maximum lateral acceleration must be positive");
  }
}

void ReferenceTrajectoryGenerator::setMaxLonAcc(const double max_lon_acc)
{
  if (max_lon_acc > 0.0)
  {
    max_lon_acc_ = max_lon_acc;
  }
  else
  {
    throw std::invalid_argument("Maximum longitudinal acceleration must be positive");
  }
}

void ReferenceTrajectoryGenerator::setMaxLonDec(const double max_lon_dec)
{
  if (max_lon_dec > 0.0)
  {
    max_lon_dec_ = max_lon_dec;
  }
  else
  {
    throw std::invalid_argument("Maximum longitudinal deceleration must be positive");
  }
}
