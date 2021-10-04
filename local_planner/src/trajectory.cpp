#include <algorithm>
#include <string>
#include <vector>

#include <visualization_msgs/MarkerArray.h>

#include "local_planner/path.h"
#include "local_planner/trajectory.h"

Trajectory::Trajectory()
{
}

Trajectory::Trajectory(const Path& path, const CubicVelocityTimeProfile& profile) : Path(path)
{
  velocity_ = std::vector<double>(size_);
  time_ = std::vector<double>(size_);

  std::transform(distance_.begin(), distance_.end(), time_.begin(),
                 [&profile](double s) { return profile.getTimeAtDisplacement(s); });

  std::transform(time_.begin(), time_.end(), velocity_.begin(),
                 [&profile](double t) { return profile.getVelocityAtTime(t); });
}

Trajectory::Trajectory(const Path& path, const AccelerationRegulator& regulator) : Path(path)
{
  velocity_ = regulator.generateVelocityProfile(path);

  time_.push_back(0.0);

  for (int i = 1; i < size_; ++i)
  {
    double dt = estimateTravelTime(distance_.at(i) - distance_.at(i - 1), velocity_.at(i - 1), velocity_.at(i));
    time_.push_back(time_.at(i - 1) + dt);
  }
}

Trajectory::Trajectory(const Path& path, const std::vector<double>& velocity, const std::vector<double>& time)
  : Path(path)
{
  velocity_ = velocity;
  time_ = time;
}

Trajectory Trajectory::trim(const size_t begin, const size_t end) const
{
  return Trajectory(Path::trim(begin, end), trimVector(velocity_, begin, end), trimVector(time_, begin, end));
}

size_t Trajectory::getWpIdAtTime(const double target_time) const
{
  for (int i = 0; i < size_; ++i)
  {
    if (time_.at(i) > target_time)
    {
      return i;
    }
  }

  return size_ - 1;
}

double Trajectory::velocity(size_t n) const
{
  return velocity_.at(n);
}

double Trajectory::time(size_t n) const
{
  return time_.at(n);
}

Trajectory& Trajectory::transform(const std::string target_frame)
{
  Path::transform(target_frame);

  return *this;
}

visualization_msgs::MarkerArray Trajectory::generateVelocityMarkers(int marker_id, const std::string& ns,
                                                                    const double scale, const double z_offset,
                                                                    const double r, const double g, const double b,
                                                                    const double a) const
{
  visualization_msgs::MarkerArray velocity_markers;

  for (int i = 0; i < size_; ++i)
  {
    visualization_msgs::Marker velocity_marker;

    velocity_marker.header.frame_id = frame_id_;
    velocity_marker.action = visualization_msgs::Marker::ADD;
    velocity_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    velocity_marker.ns = ns;
    velocity_marker.id = marker_id++;
    velocity_marker.lifetime = ros::Duration(0.1);
    velocity_marker.scale.z = scale;
    velocity_marker.color.r = r;
    velocity_marker.color.g = g;
    velocity_marker.color.b = b;
    velocity_marker.color.a = a;

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2) << velocity_.at(i);
    velocity_marker.text = oss.str();

    geometry_msgs::Point text_point = point(i);
    text_point.z += z_offset;

    velocity_marker.pose.position = text_point;
    velocity_markers.markers.push_back(velocity_marker);
  }

  return velocity_markers;
}

double Trajectory::estimateTravelTime(const double s, const double v_i, const double v_f)
{
  return 2 * s / (v_i + v_f);
}
