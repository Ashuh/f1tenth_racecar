#include <string>
#include <vector>

#include <visualization_msgs/MarkerArray.h>

#include "local_planner/path.h"
#include "local_planner/trajectory.h"

Trajectory::Trajectory()
{
}

Trajectory::Trajectory(const Path& path, const std::vector<double>& velocity) : Path(path)
{
  velocity_ = velocity;
}

double Trajectory::velocity(size_t n) const
{
  return velocity_.at(n);
}

visualization_msgs::MarkerArray Trajectory::toTextMarker(int marker_id, const std::string& ns, const double scale,
                                                         const double z_offset, const double r, const double g,
                                                         const double b, const double a) const
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
