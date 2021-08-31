#ifndef LOCAL_PLANNER_TRAJECTORY_H
#define LOCAL_PLANNER_TRAJECTORY_H

#include <string>
#include <vector>

#include <visualization_msgs/MarkerArray.h>

#include "local_planner/path.h"

class Trajectory : public Path
{
private:
  std::vector<double> velocity_;

public:
  Trajectory();

  Trajectory(const Path& path, const std::vector<double>& velocity);

  Trajectory trim(const size_t begin, const size_t end) const;

  double velocity(size_t n) const;

  IteratorPair velocityIt() const;

  visualization_msgs::MarkerArray generateVelocityMarkers(int marker_id, const std::string& ns, const double scale,
                                                          const double z_offset, const double r, const double g,
                                                          const double b, const double a = 1.0) const;
};

#endif  // LOCAL_PLANNER_TRAJECTORY_H
