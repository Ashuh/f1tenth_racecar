#ifndef LOCAL_PLANNER_TRAJECTORY_EVALUATOR_H
#define LOCAL_PLANNER_TRAJECTORY_EVALUATOR_H

#include <string>

#include <geometry_msgs/Point.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <tf2_ros/transform_listener.h>

#include "local_planner/trajectory.h"

class TrajectoryEvaluator
{
private:
  static constexpr unsigned int K_OFFSET_ = 1;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  grid_map::GridMap costmap_;

  bool checkCollision(const grid_map::Position& pos);

  grid_map::Position getWaypointPositionInMap(const double x, const double y, const std::string& wp_frame_id);

public:
  TrajectoryEvaluator();

  void setCostmap(const grid_map_msgs::GridMap::ConstPtr& costmap_msg);

  double evaluateTrajectory(const Trajectory& trajectory, const double offset);
};

#endif  // LOCAL_PLANNER_TRAJECTORY_EVALUATOR_H
