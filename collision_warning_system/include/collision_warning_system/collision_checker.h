#ifndef COLLISION_WARNING_SYSTEM_COLLISION_CHECKER_H
#define COLLISION_WARNING_SYSTEM_COLLISION_CHECKER_H

#include <vector>

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>

#include "f1tenth_msgs/ObstacleArray.h"

namespace f1tenth_racecar
{
namespace safety
{
class CollisionChecker
{
private:
  const double vehicle_length_;
  const double vehicle_width_;
  const double base_link_to_center_dist_;

  f1tenth_msgs::RectangleStamped vehicle_footprint_;
  double collision_index_;

  static geometry_msgs::Vector3 pointsToEdge(const geometry_msgs::Point p_1, const geometry_msgs::Point p_2);

  static geometry_msgs::Vector3 pointToVector(const geometry_msgs::Point point);

  static geometry_msgs::Vector3 perpendicular(const geometry_msgs::Vector3 vector);

  static std::vector<geometry_msgs::Vector3> getEdges(const f1tenth_msgs::Rectangle rect_1,
                                                      const f1tenth_msgs::Rectangle rect_2);

  static std::vector<geometry_msgs::Vector3> getAxes(const std::vector<geometry_msgs::Vector3> edges);

  static double dotProduct(const geometry_msgs::Vector3 v_1, const geometry_msgs::Vector3 v_2);

  static void projectRectOnAxis(const f1tenth_msgs::Rectangle rect, const geometry_msgs::Vector3 axis,
                                double& projection_begin, double& projection_end);

  static bool checkRectOverlap(const f1tenth_msgs::Rectangle rect_1, const f1tenth_msgs::Rectangle rect_2);

  static bool checkProjectionOverlap(const double rect_1_min, const double rect_1_max, const double rect_2_min,
                                     const double rect_2_max);

  static bool checkPointInRect(const geometry_msgs::Point point, const f1tenth_msgs::Rectangle rect);

  void computeVehicleFootprint(const geometry_msgs::PoseStamped vehicle_pose);

public:
  CollisionChecker(const double vehicle_width, const double vehicle_length, const double base_link_to_center_dist);

  bool collisionCheck(const geometry_msgs::PoseStamped vehicle_pose, const f1tenth_msgs::ObstacleArray obstacles);

  void getCollisionInfo(f1tenth_msgs::RectangleStamped& footprint, int& collision_index);
};
}  // namespace safety
}  // namespace f1tenth_racecar

#endif  // COLLISION_WARNING_SYSTEM_COLLISION_CHECKER_H
