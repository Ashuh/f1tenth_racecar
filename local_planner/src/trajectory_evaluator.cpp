#include <limits>
#include <string>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include "costmap_generator/costmap_layer.h"
#include "costmap_generator/costmap_value.h"
#include "local_planner/trajectory.h"
#include "local_planner/trajectory_evaluator.h"

TrajectoryEvaluator::TrajectoryEvaluator() : tf_listener_(tf_buffer_)
{
}

void TrajectoryEvaluator::setCostmap(const grid_map_msgs::GridMap::ConstPtr& costmap_msg)
{
  grid_map::GridMapRosConverter::fromMessage(*costmap_msg, costmap_);
}

double TrajectoryEvaluator::evaluateTrajectory(const Trajectory& trajectory, const double offset)
{
  for (int i = 0; i < trajectory.size(); ++i)
  {
    grid_map::Position pos = getWaypointPositionInMap(trajectory.x(i), trajectory.y(i), trajectory.getFrameId());

    if (checkCollision(pos))
    {
      return std::numeric_limits<double>::max();
    }
  }

  return K_OFFSET_ * abs(offset);
}

bool TrajectoryEvaluator::checkCollision(const grid_map::Position& pos)
{
  if (!costmap_.exists(CostmapLayer::INFLATION))
  {
    throw std::runtime_error(CostmapLayer::INFLATION + " layer is not available in costmap");
  }

  if (!costmap_.isInside(pos))
  {
    return true;
  }

  return costmap_.atPosition(CostmapLayer::INFLATION, pos) == static_cast<int>(CostmapValue::OCCUPIED);
}

grid_map::Position TrajectoryEvaluator::getWaypointPositionInMap(const double x, const double y,
                                                                 const std::string& wp_frame_id)
{
  geometry_msgs::TransformStamped tf = tf_buffer_.lookupTransform(costmap_.getFrameId(), wp_frame_id, ros::Time(0));

  geometry_msgs::Point point;
  point.x = x;
  point.y = y;
  tf2::doTransform(point, point, tf);

  return grid_map::Position(point.x, point.y);
}
