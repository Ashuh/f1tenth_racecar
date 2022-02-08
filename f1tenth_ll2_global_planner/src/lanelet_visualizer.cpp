#include "f1tenth_ll2_global_planner/lanelet_visualizer.h"

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/LaneletMap.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>

LaneletVisualizer::LaneletVisualizer(const std::string& lanelet_frame_id)
{
  lanelet_frame_id_ = lanelet_frame_id;
}

visualization_msgs::Marker LaneletVisualizer::generateLineStringMarker(const lanelet::ConstLineString3d& line_string,
                                                                       const std::string& ns, const int id,
                                                                       const std_msgs::ColorRGBA& color) const
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = lanelet_frame_id_;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.01;
  marker.lifetime = ros::Duration(0);
  marker.pose.orientation.w = 1.0;
  marker.color = color;

  for (auto& ll2_point : line_string)
  {
    geometry_msgs::Point point_msg;
    point_msg.x = ll2_point.x();
    point_msg.y = ll2_point.y();
    marker.points.push_back(point_msg);
  }

  return marker;
}

visualization_msgs::MarkerArray LaneletVisualizer::generateMapMarker(const lanelet::LaneletMapPtr& map) const
{
  std_msgs::ColorRGBA black;
  black.r = 0;
  black.g = 0;
  black.b = 0;
  black.a = 1;
  std_msgs::ColorRGBA white;
  white.r = 1;
  white.g = 1;
  white.b = 1;
  white.a = 1;

  int id = 0;
  visualization_msgs::MarkerArray markers;

  for (const auto& l : map->laneletLayer)
  {
    markers.markers.push_back(generateLineStringMarker(l.leftBound(), lanelet_frame_id_, id++, black));
    markers.markers.push_back(generateLineStringMarker(l.rightBound(), lanelet_frame_id_, id++, black));
    markers.markers.push_back(generateLineStringMarker(l.centerline(), lanelet_frame_id_, id++, white));
  }

  return markers;
}