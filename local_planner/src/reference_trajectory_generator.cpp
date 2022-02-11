#include "local_planner/reference_trajectory_generator.h"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>

#include <algorithm>
#include <unsupported/Eigen/Splines>
#include <utility>
#include <vector>

#include "f1tenth_utils/math.h"
#include "local_planner/acceleration_regulator.h"
#include "local_planner/lattice.h"
#include "local_planner/path.h"
#include "local_planner/trajectory.h"

ReferenceTrajectoryGenerator::ReferenceTrajectoryGenerator(
    const std::shared_ptr<Lattice::Generator>& lattice_generator_ptr,
    const std::shared_ptr<AccelerationRegulator>& acc_regulator_ptr,
    const std::shared_ptr<visualization_msgs::MarkerArray>& viz_ptr)
{
  lat_gen_ptr_ = lattice_generator_ptr;
  acc_regulator_ptr_ = acc_regulator_ptr;
  viz_ptr_ = viz_ptr;
}

Trajectory ReferenceTrajectoryGenerator::generateReferenceTrajectory(const geometry_msgs::Pose& current_pose)
{
  Lattice lattice = lat_gen_ptr_->generate(current_pose);
  visualizeLattice(lattice);
  Path reference_path = generateReferencePath(lattice);
  acc_regulator_ptr_->setZeroFinalVelocity(lattice.isEndOfPath());
  Trajectory reference_trajectory(reference_path, *acc_regulator_ptr_);
  visualizeReferenceTrajectory(reference_trajectory);

  return reference_trajectory;
}

Path ReferenceTrajectoryGenerator::generateReferencePath(Lattice& lattice)
{
  lattice.computeShortestPaths();

  // Get SSSP for each vertex in the furthest possible layer
  static constexpr int MIN_PATH_SIZE = 2;
  int max_offset = (lattice.getNumLateralSamples() - 1) / 2;

  std::vector<std::pair<std::vector<geometry_msgs::Point>, double>> sssp_results;

  for (int i = lattice.getNumLayers() - 1; i >= MIN_PATH_SIZE - 1; --i)
  {
    for (int j = -max_offset; j <= max_offset; ++j)
    {
      auto result = lattice.getShortestPath(i, j);

      if (result)
      {
        sssp_results.push_back(result.get());
      }
    }

    if (!sssp_results.empty())
    {
      break;
    }
  }

  if (sssp_results.empty())
  {
    throw std::runtime_error("Failed to find path in lattice");
  }

  std::vector<geometry_msgs::Point> best_sssp = getBestSSSP(sssp_results);
  visualizeSSSP(best_sssp);

  static constexpr int MIN_CUBIC_INTERPOLATION_POINTS = 4;  // Cubic spline interpolation requires at least 4 points
  std::vector<geometry_msgs::Point> interpolated_sssp = best_sssp.size() < MIN_CUBIC_INTERPOLATION_POINTS ?
                                                            linearInterpolate(best_sssp, 0.1) :
                                                            cubicSplineInterpolate(best_sssp);
  Path reference_path = pointsToPath(interpolated_sssp);
  return reference_path;
}

std::vector<geometry_msgs::Point> ReferenceTrajectoryGenerator::getBestSSSP(
    std::vector<std::pair<std::vector<geometry_msgs::Point>, double>>& sssp_results)
{
  std::sort(sssp_results.begin(), sssp_results.end(),
            [](const auto& p1, const auto& p2) { return p1.second < p2.second; });
  return sssp_results.at(0).first;
}

std::vector<geometry_msgs::Point>
ReferenceTrajectoryGenerator::cubicSplineInterpolate(const std::vector<geometry_msgs::Point>& path)
{
  Eigen::ArrayXXd points(2, path.size());

  for (int i = 0; i < path.size(); i++)
  {
    geometry_msgs::Point point = path.at(i);
    points(0, i) = point.x;
    points(1, i) = point.y;
  }

  const Eigen::Spline<double, 2> spline = Eigen::SplineFitting<Eigen::Spline<double, 2>>::Interpolate(points, 3);
  int num_samples = 100;
  double step = 1.0 / (num_samples - 1);
  std::vector<geometry_msgs::Point> spline_points;

  for (int i = 0; i < num_samples; ++i)
  {
    Eigen::Spline<double, 2>::PointType p = spline(i * step);

    geometry_msgs::Point point;
    point.x = p(0, 0);
    point.y = p(1, 0);

    spline_points.push_back(point);
  }

  return spline_points;
}

std::vector<geometry_msgs::Point> ReferenceTrajectoryGenerator::linearInterpolate(const geometry_msgs::Point& from,
                                                                                  const geometry_msgs::Point& to,
                                                                                  const double step_size)
{
  double distance = calculateDistance(from.x, from.y, to.x, to.y);
  double dx = to.x - from.x;
  double dy = to.y - from.y;
  double angle = atan2(dy, dx);
  double step_x = step_size * cos(angle);
  double step_y = step_size * sin(angle);

  std::vector<geometry_msgs::Point> points;
  points.push_back(from);

  int num_via_points = std::ceil(distance / step_size) - 1;

  for (int i = 1; i <= num_via_points; ++i)
  {
    geometry_msgs::Point point = from;
    point.x += i * step_x;
    point.y += i * step_y;
    points.push_back(point);
  }

  points.push_back(to);
  return points;
}

std::vector<geometry_msgs::Point>
ReferenceTrajectoryGenerator::linearInterpolate(const std::vector<geometry_msgs::Point>& points, const double step_size)
{
  std::vector<geometry_msgs::Point> interpolated_points;

  for (int i = 0; i < points.size() - 1; ++i)
  {
    geometry_msgs::Point from = points.at(i);
    geometry_msgs::Point to = points.at(i + 1);
    std::vector<geometry_msgs::Point> interpolated_section = linearInterpolate(from, to, step_size);
    interpolated_points.insert(interpolated_points.end(), interpolated_section.begin(), interpolated_section.end());
  }

  return interpolated_points;
}

Path ReferenceTrajectoryGenerator::pointsToPath(std::vector<geometry_msgs::Point> points)
{
  std::vector<double> distance_vec;
  std::vector<double> x_vec;
  std::vector<double> y_vec;
  std::vector<double> yaw_vec;
  std::vector<double> curvature_vec;

  for (int i = 0; i < points.size(); ++i)
  {
    distance_vec.push_back((i == 0) ? 0.0 : distance_vec.at(i - 1) + distance(points.at(i), points.at(i - 1)));

    if (i == points.size() - 1)
    {
      yaw_vec.push_back(yaw_vec.at(i - 1));
    }
    else
    {
      double dx = points.at(i + 1).x - points.at(i).x;
      double dy = points.at(i + 1).y - points.at(i).y;
      double yaw = atan2(dy, dx);
      yaw_vec.push_back(yaw);
    }

    x_vec.push_back(points.at(i).x);
    y_vec.push_back(points.at(i).y);

    if (i == 0)
    {
      curvature_vec.push_back(mengerCurvature(points.at(i), points.at(i + 1), points.at(i + 2)));
    }
    else if (i == points.size() - 1)
    {
      curvature_vec.push_back(mengerCurvature(points.at(i - 2), points.at(i - 1), points.at(i)));
    }
    else
    {
      curvature_vec.push_back(mengerCurvature(points.at(i - 1), points.at(i), points.at(i + 1)));
    }
  }

  return Path("map", distance_vec, x_vec, y_vec, yaw_vec, curvature_vec);
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

void ReferenceTrajectoryGenerator::visualizeLattice(const Lattice& lattice)
{
  if (viz_ptr_ != nullptr)
  {
    visualization_msgs::Marker vertex_marker = lattice.generateVertexMarker(0, "lattice/vertices", 0.02, 1, 1, 1);
    visualization_msgs::Marker edge_marker = lattice.generateEdgeMarker(0, "lattice/edges", 0.002, 0, 0, 1, 0.7);
    visualization_msgs::MarkerArray weight_markers = lattice.generateWeightMarkers(0, "lattice/weights", 0.02, 0, 0, 1);

    visualization_msgs::MarkerArray lattice_marker;
    lattice_marker.markers = { vertex_marker, edge_marker };
    lattice_marker.markers.insert(lattice_marker.markers.end(), weight_markers.markers.begin(),
                                  weight_markers.markers.end());

    viz_ptr_->markers.insert(viz_ptr_->markers.end(), lattice_marker.markers.begin(), lattice_marker.markers.end());
  }
}

void ReferenceTrajectoryGenerator::visualizeSSSP(const std::vector<geometry_msgs::Point>& path)
{
  if (viz_ptr_ == nullptr)
  {
    return;
  }

  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "sssp";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.02;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  for (const auto& p : path)
  {
    marker.points.push_back(p);
  }

  viz_ptr_->markers.push_back(marker);
}

void ReferenceTrajectoryGenerator::visualizeReferenceTrajectory(const Trajectory& trajectory)
{
  if (viz_ptr_ == nullptr)
  {
    return;
  }

  visualization_msgs::Marker path_marker =
      trajectory.generateLineMarker(0, "reference_trajectory/path", 0.02, 0.0, 0.0, 0.0, 0.0);
  visualization_msgs::MarkerArray velocity_markers =
      trajectory.generateVelocityMarkers(0, "reference_trajectory/velocity", 0.05, 0.1, 0.0, 0.0, 0.0);

  viz_ptr_->markers.push_back(path_marker);
  viz_ptr_->markers.insert(viz_ptr_->markers.begin(), velocity_markers.markers.begin(), velocity_markers.markers.end());
}
