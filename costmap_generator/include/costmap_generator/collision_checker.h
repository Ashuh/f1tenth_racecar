#ifndef COSTMAP_GENERATOR_COLLISION_CHECKER_H
#define COSTMAP_GENERATOR_COLLISION_CHECKER_H

#include <string>
#include <vector>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>

class CollisionChecker
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber costmap_sub_;

  grid_map::GridMap costmap_;           // latest available costmap
  std::string layer_id_ = "INFLATION";  // id of the layer to use in the costmap
  std::vector<double> circle_offsets_;  // offsets of the circle centers with respect to the base link

  void costmapCallback(const grid_map_msgs::GridMap::ConstPtr& costmap_msg);

  std::vector<geometry_msgs::PointStamped>
  getCirclePositionsFromPose(const geometry_msgs::PoseStamped& pose_stamped) const;

  std::vector<geometry_msgs::PoseStamped> lineToPoses(const geometry_msgs::PointStamped& source,
                                                      const geometry_msgs::PointStamped& target) const;

public:
  explicit CollisionChecker(const std::vector<double>& circle_offsets);

  /**
   * @brief Checks whether a pose would result in a collision. The pose will be transformed to the same frame as the
   * costmap as long as the frame id in the header is correctly specified.
   *
   * @param pose The pose to check for collision.
   * @return The cost associated with the pose. CostmapValue::OCCUPIED indicates a collision.
   */
  double checkCollision(const geometry_msgs::PoseStamped& pose_stamped) const;

  double checkCollision(const geometry_msgs::PointStamped& source, const geometry_msgs::PointStamped& target) const;
};

#endif  // COSTMAP_GENERATOR_COLLISION_CHECKER_H
