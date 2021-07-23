#ifndef LOCAL_PLANNER_REFERENCE_TRAJECTORY_GENERATOR_H
#define LOCAL_PLANNER_REFERENCE_TRAJECTORY_GENERATOR_H

#include <tuple>
#include <utility>
#include <vector>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

#include "local_planner/lattice.h"
#include "local_planner/lattice_generator.h"
#include "local_planner/path.h"
#include "local_planner/trajectory.h"

class ReferenceTrajectoryGenerator
{
private:
  double speed_limit_;
  double max_lat_acc_;
  double max_lon_acc_;
  double max_lon_dec_;

  LatticeGenerator lat_gen_;

  std::shared_ptr<visualization_msgs::MarkerArray> viz_ptr_;

  Path generateReferencePath(const geometry_msgs::Pose& current_pose);

  std::vector<geometry_msgs::Point> getBestSSSP(const std::vector<std::vector<geometry_msgs::Point>>& sssp_candidates);

  std::vector<geometry_msgs::Point> cubicSplineInterpolate(const std::vector<geometry_msgs::Point>& path);

  Path pointsToPath(std::vector<geometry_msgs::Point> points);

  std::vector<double> generateVelocityProfile(const Path& path);

  bool isValidProfile(const Path& path, const std::vector<double>& velocity_profile);

  std::vector<std::pair<int, int>> identifyRegions(const std::vector<double>& velocity_profile);

  /**
   * @brief Calculates the acceleration required to accelerate from an initial velocity to a final velocity over a
   * certain distance.
   *
   * @param v_i Initial velocity.
   * @param v_f Final velocity.
   * @param s Distance.
   * @return The required acceleration.
   */
  double getLonAcc(const double v_i, const double v_f, const double s);

  /**
   * @brief Calculates the final velocity given the initial velocity, acceleration, and distance travelled.
   *
   * @param v_i Initial velocity.
   * @param a Acceleration.
   * @param s Distance.
   * @return The final velocity.
   */
  double getFinalVelocity(const double v_i, const double a, const double s);

  double distance(const geometry_msgs::Point& a, const geometry_msgs::Point& b);

  /**
   * @brief Calculates the area of a triangle given the length of each side (Heron's formula).
   *
   * @param length_ab Length of the edge between vertex a and b.
   * @param length_bc Length of the edge between vertex b and c.
   * @param length_ca Length of the edge between vertex c and a.
   * @return The area of the triangle.
   */
  double triangleArea(const double length_ab, const double length_bc, const double length_ca);

  /**
   * @brief Calculates the reciprocal of the radius of the circle that passes through 3 points (Menger curvature).
   *
   * @param a Point a.
   * @param b Point b.
   * @param c Point c.
   * @return The Menger curvature of the 3 points.
   */
  double mengerCurvature(const geometry_msgs::Point& a, const geometry_msgs::Point& b, const geometry_msgs::Point& c);

  void visualizeLattice(const Lattice& lattice);

  void visualizeSSSP(const Path& path);

  void visualizeReferenceTrajectory(const Trajectory& trajectory);

public:
  ReferenceTrajectoryGenerator(const int num_layers, const double longitudinal_spacing, const int num_lateral_samples,
                               const double lateral_spacing, const double k_length, const double speed_limit,
                               const double max_lat_acc, const double max_lon_acc, const double max_lon_dec,
                               const std::shared_ptr<visualization_msgs::MarkerArray>& viz_ptr = nullptr);

  Trajectory generateReferenceTrajectory(const geometry_msgs::Pose& current_pose);

  void setGlobalPath(const nav_msgs::PathConstPtr& global_path_msg);

  void setCostmap(const grid_map_msgs::GridMap::ConstPtr& costmap_msg);

  void setSpeedLimit(const double speed_limit);

  void setMaxLatAcc(const double max_lat_acc);

  void setMaxLonAcc(const double max_lon_acc);

  void setMaxLonDec(const double max_lon_dec);
};

#endif  // LOCAL_PLANNER_REFERENCE_TRAJECTORY_GENERATOR_H
