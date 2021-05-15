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
  geometry_msgs::Point front_left_;
  geometry_msgs::Point front_right_;
  geometry_msgs::Point rear_left_;
  geometry_msgs::Point rear_right_;

  geometry_msgs::PolygonStamped vehicle_footprint_;
  double collision_index_;

  static geometry_msgs::Vector3 pointsToEdge(const geometry_msgs::Point32 p_1, const geometry_msgs::Point32 p_2);

  static geometry_msgs::Vector3 pointToVector(const geometry_msgs::Point32 point);

  static geometry_msgs::Vector3 perpendicular(const geometry_msgs::Vector3 vector);

  static std::vector<geometry_msgs::Vector3> getEdges(const geometry_msgs::Polygon rect_1,
                                                      const geometry_msgs::Polygon rect_2);

  static std::vector<geometry_msgs::Vector3> getAxes(const std::vector<geometry_msgs::Vector3> edges);

  static double dotProduct(const geometry_msgs::Vector3 v_1, const geometry_msgs::Vector3 v_2);

  static void projectRectOnAxis(const geometry_msgs::Polygon rect, const geometry_msgs::Vector3 axis,
                                double& projection_begin, double& projection_end);

  static bool checkRectOverlap(const geometry_msgs::Polygon rect_1, const geometry_msgs::Polygon rect_2);

  static bool checkProjectionOverlap(const double rect_1_min, const double rect_1_max, const double rect_2_min,
                                     const double rect_2_max);

  static bool checkPointInRect(const geometry_msgs::Point32 point, const geometry_msgs::Polygon rect);

  geometry_msgs::Polygon transformVehicleRect(const geometry_msgs::Pose vehicle_pose);

public:
  CollisionChecker(const double vehicle_width, const double vehicle_length, const double rear_axle_from_cg);

  bool collisionCheck(const geometry_msgs::Pose vehicle_pose, const f1tenth_msgs::ObstacleArray obstacles);

  void getCollisionInfo(geometry_msgs::PolygonStamped& footprint, int& collision_index);
};
}  // namespace safety
}  // namespace f1tenth_racecar

#endif  // COLLISION_WARNING_SYSTEM_COLLISION_CHECKER_H
