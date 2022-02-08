#ifndef F1TENTH_LL2_GLOBAL_PLANNER_GLOBAL_PLANNER_H
#define F1TENTH_LL2_GLOBAL_PLANNER_GLOBAL_PLANNER_H

#include <geometry_msgs/PoseStamped.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_projection/UTM.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>
#include <vector>

#include "f1tenth_ll2_global_planner/lanelet_visualizer.h"
#include "f1tenth_utils/math.h"
#include "f1tenth_utils/tf2_wrapper.h"

class GlobalPlanner
{
public:
  GlobalPlanner();

private:
  ros::NodeHandle nh_;

  ros::Publisher map_viz_pub_;
  ros::Publisher path_pub_;

  ros::Subscriber goal_sub_;
  ros::Subscriber odom_sub_;

  lanelet::LaneletMapPtr map_;
  geometry_msgs::PoseStamped cur_pose_;

  std::string lanelet_frame_id_ = "earth";
  std::string map_frame_id_ = "map";

  LaneletVisualizer visualizer_;

  void goalCallback(const geometry_msgs::PoseStamped& pose);

  void odomCallback(const nav_msgs::Odometry& pose);

  void loadMap(const std::string& file, double lat, double lon);

  nav_msgs::Path plan(const lanelet::ConstLanelet& from, const lanelet::ConstLanelet& to);

  static std::vector<geometry_msgs::PoseStamped> interpolatePoses(const geometry_msgs::PoseStamped& from,
                                                                  const geometry_msgs::PoseStamped& to,
                                                                  double step_size);

  std::vector<geometry_msgs::PoseStamped> lineStringToPoses(const lanelet::ConstLineString3d& line_string) const;
};

#endif  // F1TENTH_LL2_GLOBAL_PLANNER_GLOBAL_PLANNER_H
