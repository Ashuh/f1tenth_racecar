#ifndef F1TENTH_LL2_GLOBAL_PLANNER_LANELET_VISUALIZER_H
#define F1TENTH_LL2_GLOBAL_PLANNER_LANELET_VISUALIZER_H

#include <lanelet2_core/Forward.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>

class LaneletVisualizer
{
public:
  LaneletVisualizer() = default;

  explicit LaneletVisualizer(const std::string& lanelet_frame_id);

  visualization_msgs::MarkerArray generateMapMarker(const lanelet::LaneletMapPtr& map) const;

private:
  std::string lanelet_frame_id_;

  visualization_msgs::Marker generateLineStringMarker(const lanelet::ConstLineString3d& line_string,
                                                      const std::string& ns, const int id,
                                                      const std_msgs::ColorRGBA& color) const;
};

#endif  // F1TENTH_LL2_GLOBAL_PLANNER_LANELET_VISUALIZER_H
