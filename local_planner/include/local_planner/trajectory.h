#ifndef LOCAL_PLANNER_TRAJECTORY_H
#define LOCAL_PLANNER_TRAJECTORY_H

#include <string>
#include <vector>

#include <visualization_msgs/MarkerArray.h>

#include "f1tenth_msgs/Trajectory.h"
#include "local_planner/acceleration_regulator.h"
#include "local_planner/cubic_velocity_time_profile.h"
#include "local_planner/path.h"

class Trajectory : public Path
{
private:
  std::vector<double> velocity_;

  std::vector<double> time_;

  Trajectory(const Path& path, const std::vector<double>& velocity, const std::vector<double>& time);

  static double estimateTravelTime(const double s, const double v_i, const double v_f);

public:
  Trajectory();

  Trajectory(const Path& path, const CubicVelocityTimeProfile& profile);

  Trajectory(const Path& path, const AccelerationRegulator& regulator);

  Trajectory trim(const size_t begin, const size_t end) const;

  size_t getWpIdAtTime(const double target_time) const;

  double velocity(size_t n) const;

  double time(size_t n) const;

  Trajectory& transform(const std::string target_frame) override;

  f1tenth_msgs::Trajectory toMsg() const;

  visualization_msgs::MarkerArray generateVelocityMarkers(int marker_id, const std::string& ns, const double scale,
                                                          const double z_offset, const double r, const double g,
                                                          const double b, const double a = 1.0) const;
};

#endif  // LOCAL_PLANNER_TRAJECTORY_H
