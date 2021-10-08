#include <algorithm>
#include <utility>
#include <vector>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <unsupported/Eigen/Splines>

#include "local_planner/acceleration_regulator.h"
#include "local_planner/lattice.h"
#include "local_planner/path.h"
#include "local_planner/reference_trajectory_generator.h"
#include "local_planner/trajectory.h"

ReferenceTrajectoryGenerator::ReferenceTrajectoryGenerator(
    const Lattice::Generator& lattice_generator, const AccelerationRegulator::Constraints& velocity_constraints,
    const std::shared_ptr<visualization_msgs::MarkerArray>& viz_ptr)
  : lat_gen_(lattice_generator), acc_capper_(velocity_constraints)
{
  viz_ptr_ = viz_ptr;
}

Trajectory ReferenceTrajectoryGenerator::generateReferenceTrajectory(const geometry_msgs::PoseStamped& current_pose)
{
  Path reference_path = generateReferencePath(current_pose);
  Trajectory reference_trajectory(reference_path, acc_capper_);
  visualizeReferenceTrajectory(reference_trajectory);

  return reference_trajectory;
}

Path ReferenceTrajectoryGenerator::generateReferencePath(const geometry_msgs::PoseStamped& current_pose)
{
  // Generate lattice
  Lattice lattice = lat_gen_.generate(current_pose);
  visualizeLattice(lattice);

  // Get SSSP for each vertex in final layer
  std::vector<std::vector<geometry_msgs::Point>> sssp_results;

  for (int i = 1; i < lattice.getNumLateralSamples() + 1; ++i)
  {
    int offset = ((i % 2 == 0) ? 1 : -1) * i / 2;

    std::vector<geometry_msgs::Point> sssp = lattice.getShortestPath(offset);
    sssp_results.push_back(sssp);
  }

  // Select the best SSSP
  std::vector<geometry_msgs::Point> best_sssp = getBestSSSP(sssp_results);
  visualizeSSSP(pointsToPath(best_sssp));
  Path reference_path = pointsToPath(cubicSplineInterpolate(best_sssp)).transform("base_link");
  return reference_path;
}

std::vector<geometry_msgs::Point>
ReferenceTrajectoryGenerator::getBestSSSP(const std::vector<std::vector<geometry_msgs::Point>>& sssp_candidates)
{
  // placeholder code

  std::vector<geometry_msgs::Point> best_sssp;

  for (auto& sssp : sssp_candidates)
  {
    if (!sssp.empty())
    {
      best_sssp = sssp;
      break;
    }
  }

  if (best_sssp.empty())
  {
    throw std::runtime_error("Failed to find a path to the final layer of the lattice");
  }

  return best_sssp;
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

  std::vector<geometry_msgs::Point> spline_points;

  for (double u = 0; u <= 1; u += 0.025)
  {
    Eigen::Spline<double, 2>::PointType p = spline(u);

    geometry_msgs::Point point;
    point.x = p(0, 0);
    point.y = p(1, 0);

    spline_points.push_back(point);
  }

  std::reverse(spline_points.begin(), spline_points.end());

  return spline_points;
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

void ReferenceTrajectoryGenerator::setGlobalPath(const nav_msgs::PathConstPtr& global_path_msg)
{
  lat_gen_.setGlobalPath(*global_path_msg);
}

void ReferenceTrajectoryGenerator::setCostmap(const grid_map_msgs::GridMap::ConstPtr& costmap_msg)
{
  lat_gen_.setCostmap(costmap_msg);
}

void ReferenceTrajectoryGenerator::setLengthWeight(const double weight)
{
  lat_gen_.setLengthWeight(weight);
}

void ReferenceTrajectoryGenerator::setLatticePattern(const Lattice::Generator::Pattern& pattern)
{
  lat_gen_.setPattern(pattern);
}

void ReferenceTrajectoryGenerator::setVelocityConstraints(const AccelerationRegulator::Constraints& constraints)
{
  acc_capper_.setConstraints(constraints);
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

void ReferenceTrajectoryGenerator::visualizeSSSP(const Path& path)
{
  if (viz_ptr_ == nullptr)
  {
    return;
  }

  visualization_msgs::Marker path_marker = path.generateLineMarker(0, "sssp", 0.02, 0.0, 0.0, 0.0, 1.0);
  viz_ptr_->markers.push_back(path_marker);
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
