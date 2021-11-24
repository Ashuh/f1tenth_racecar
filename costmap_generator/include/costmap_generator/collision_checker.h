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
  ros::ServiceClient client_;

  const std::string id_;                // id of the collision checker
  grid_map::GridMap costmap_;           // latest available costmap
  std::string layer_id_;                // id of the layer to use in the costmap
  std::vector<double> circle_offsets_;  // offsets of the circle centers with respect to the base link
  double circle_radius_;                // radius of the circles

  void connect();

  void sendInflationRequest(const double radius);

  void cancelInflationRequest();

  std::vector<geometry_msgs::PointStamped>
  getCirclePositionsFromPose(const geometry_msgs::PoseStamped& pose_stamped) const;

  std::vector<geometry_msgs::PoseStamped> lineToPoses(const geometry_msgs::PointStamped& source,
                                                      const geometry_msgs::PointStamped& target) const;

public:
  CollisionChecker(const std::vector<double>& circle_offsets, const double circle_radius, const std::string& id = "");

  ~CollisionChecker();

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
};

#endif  // COSTMAP_GENERATOR_COLLISION_CHECKER_H
