#ifndef LOCAL_PLANNER_ACCELERATION_REGULATOR_H
#define LOCAL_PLANNER_ACCELERATION_REGULATOR_H

#include <utility>
#include <vector>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>

#include "local_planner/path.h"

class AccelerationRegulator
{
public:
  struct Constraints
  {
    double max_speed_;
    double max_lat_acc_;
    double max_lon_acc_;
    double max_lon_dec_;

    Constraints(const double max_speed, const double max_lat_acc, const double max_lon_acc, const double max_lon_dec);
  };

  explicit AccelerationRegulator(const Constraints& constraints);

  std::vector<double> generateVelocityProfile(const Path& path);

  void setConstraints(const Constraints& constraints);

private:
  Constraints constraints_;

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
};

#endif  // LOCAL_PLANNER_ACCELERATION_REGULATOR_H
