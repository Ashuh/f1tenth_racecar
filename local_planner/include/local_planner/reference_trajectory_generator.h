#ifndef LOCAL_PLANNER_REFERENCE_TRAJECTORY_GENERATOR_H
#define LOCAL_PLANNER_REFERENCE_TRAJECTORY_GENERATOR_H

#include <utility>
#include <vector>

#include <geometry_msgs/Point.h>

#include "local_planner/path.h"
#include "local_planner/trajectory.h"

class ReferenceTrajectoryGenerator
{
private:
  double speed_limit_;
  double max_lat_acc_;
  double max_lon_acc_;
  double max_lon_dec_;

  Path generateReferencePath(const std::vector<geometry_msgs::Point>& lattice_vertices);

  std::vector<geometry_msgs::Point> cubicSplineInterpolate(const std::vector<geometry_msgs::Point>& path);

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

public:
  ReferenceTrajectoryGenerator(const double speed_limit, const double max_lat_acc, const double max_lon_acc,
                               const double max_lon_dec);

  Trajectory generateReferenceTrajectory(const std::vector<geometry_msgs::Point>& lattice_vertices);

  void setSpeedLimit(const double speed_limit);

  void setMaxLatAcc(const double max_lat_acc);

  void setMaxLonAcc(const double max_lon_acc);

  void setMaxLonDec(const double max_lon_dec);
};

#endif  // LOCAL_PLANNER_REFERENCE_TRAJECTORY_GENERATOR_H
