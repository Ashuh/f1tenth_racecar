#ifndef COSTMAP_GENERATOR_COLLISION_CHECKER_H
#define COSTMAP_GENERATOR_COLLISION_CHECKER_H

#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <visualization_msgs/MarkerArray.h>

#include "costmap_generator/costmap_layer.h"

class CollisionChecker
{
private:
  std::vector<double> circle_offsets_;  // offsets of the circle centers with respect to the base link
  double circle_radius_;                // radius of the circles

  grid_map::GridMap inflated_costmap_;

  mutable std::mutex mutex_;

  /**
   * @brief Inflates the occupied cells in the costmap such that all cells within the specified inflation radius are
   * marked as occupied as well.
   *
   * @param costmap The map to be inflated.
   * @param radius The inflation radius.
   * @return The inflated map.
   */
  static grid_map::GridMap inflateMap(const grid_map::GridMap& costmap, const double radius);

  std::vector<geometry_msgs::PointStamped>
  getCirclePositionsFromPose(const geometry_msgs::PoseStamped& pose_stamped) const;

  std::vector<geometry_msgs::PoseStamped> lineToPoses(const geometry_msgs::PointStamped& source,
                                                      const geometry_msgs::PointStamped& target) const;

public:
  CollisionChecker(const std::vector<double>& circle_offsets, const double circle_radius);

  /**
   * @brief Checks whether a pose would result in a collision. The pose will be transformed to the same frame as the
   * costmap as long as the frame id in the header is correctly specified.
   *
   * @param pose The pose to check for collision.
   * @return True if there is a collision.
   */
  bool checkCollision(const geometry_msgs::PoseStamped& pose_stamped) const;

  bool checkCollision(const geometry_msgs::PointStamped& source, const geometry_msgs::PointStamped& target) const;

  void setCostmap(const grid_map_msgs::GridMap::ConstPtr& costmap_msg);

  nav_msgs::OccupancyGrid getInflatedGridMsg() const;
};

#endif  // COSTMAP_GENERATOR_COLLISION_CHECKER_H
