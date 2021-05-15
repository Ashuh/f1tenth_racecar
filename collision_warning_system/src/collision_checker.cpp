#include <algorithm>
#include <vector>

#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "f1tenth_msgs/ObstacleArray.h"
#include "collision_warning_system/collision_checker.h"

namespace f1tenth_racecar
{
namespace safety
{
void CollisionChecker::computeVehicleFootprint(const geometry_msgs::PoseStamped vehicle_pose)
{
  geometry_msgs::TransformStamped tf;
  tf.transform.translation.x = vehicle_pose.pose.position.x;
  tf.transform.translation.y = vehicle_pose.pose.position.y;
  tf.transform.rotation = vehicle_pose.pose.orientation;

  vehicle_footprint_.header.frame_id = vehicle_pose.header.frame_id;

  geometry_msgs::Point front_left;
  geometry_msgs::Point front_right;
  geometry_msgs::Point rear_right;
  geometry_msgs::Point rear_left;

  front_left.x = (vehicle_length_ / 2) + base_link_to_center_dist_;
  front_left.y = -vehicle_width_ / 2;

  front_right.x = (vehicle_length_ / 2) + base_link_to_center_dist_;
  front_right.y = vehicle_width_ / 2;

  rear_right.x = (vehicle_length_ / 2) - base_link_to_center_dist_;
  rear_right.y = vehicle_width_ / 2;

  rear_left.x = (vehicle_length_ / 2) - base_link_to_center_dist_;
  rear_left.y = -vehicle_width_ / 2;

  tf2::doTransform(front_left, vehicle_footprint_.rectangle.a, tf);
  tf2::doTransform(front_right, vehicle_footprint_.rectangle.b, tf);
  tf2::doTransform(rear_right, vehicle_footprint_.rectangle.c, tf);
  tf2::doTransform(rear_left, vehicle_footprint_.rectangle.d, tf);
}

geometry_msgs::Vector3 CollisionChecker::pointsToEdge(const geometry_msgs::Point p_1, const geometry_msgs::Point p_2)
{
  geometry_msgs::Vector3 edge;
  edge.x = p_2.x - p_1.x;
  edge.y = p_2.y - p_1.y;

  return edge;
}

geometry_msgs::Vector3 CollisionChecker::pointToVector(const geometry_msgs::Point point)
{
  geometry_msgs::Vector3 vector;
  vector.x = point.x;
  vector.y = point.y;

  return vector;
}

geometry_msgs::Vector3 CollisionChecker::perpendicular(const geometry_msgs::Vector3 vector)
{
  geometry_msgs::Vector3 perpendicular;
  perpendicular.x = vector.y;
  perpendicular.y = -vector.x;

  return perpendicular;
}

std::vector<geometry_msgs::Vector3> CollisionChecker::getEdges(const f1tenth_msgs::Rectangle rect_1,
                                                               const f1tenth_msgs::Rectangle rect_2)
{
  std::vector<geometry_msgs::Vector3> edges;

  edges.push_back(pointsToEdge(rect_1.a, rect_1.b));
  edges.push_back(pointsToEdge(rect_1.b, rect_1.c));

  edges.push_back(pointsToEdge(rect_2.a, rect_2.b));
  edges.push_back(pointsToEdge(rect_2.b, rect_2.c));

  return edges;
}

std::vector<geometry_msgs::Vector3> CollisionChecker::getAxes(const std::vector<geometry_msgs::Vector3> edges)
{
  std::vector<geometry_msgs::Vector3> axes(edges.size());
  std::transform(edges.begin(), edges.end(), axes.begin(), perpendicular);

  return axes;
}

double CollisionChecker::dotProduct(const geometry_msgs::Vector3 v_1, const geometry_msgs::Vector3 v_2)
{
  return v_1.x * v_2.x + v_1.y * v_2.y;
}

void CollisionChecker::projectRectOnAxis(const f1tenth_msgs::Rectangle rect, const geometry_msgs::Vector3 axis,
                                         double& projection_begin, double& projection_end)
{
  std::vector<double> projections = { dotProduct(pointToVector(rect.a), axis), dotProduct(pointToVector(rect.b), axis),
                                      dotProduct(pointToVector(rect.c), axis),
                                      dotProduct(pointToVector(rect.d), axis) };

  projection_begin = *std::min_element(projections.begin(), projections.end());
  projection_end = *std::max_element(projections.begin(), projections.end());
}

bool CollisionChecker::checkPointInRect(const geometry_msgs::Point point, const f1tenth_msgs::Rectangle rect)
{
  geometry_msgs::Vector3 edge_ab = pointsToEdge(rect.a, rect.b);
  geometry_msgs::Vector3 edge_bc = pointsToEdge(rect.b, rect.c);

  geometry_msgs::Vector3 point_vec;
  point_vec.x = point.x;
  point_vec.y = point.y;

  double a_ab = dotProduct(pointToVector(rect.a), edge_ab);
  double b_ab = dotProduct(pointToVector(rect.b), edge_ab);
  double point_ab = dotProduct(pointToVector(point), edge_ab);

  double b_bc = dotProduct(pointToVector(rect.b), edge_bc);
  double c_bc = dotProduct(pointToVector(rect.c), edge_bc);
  double point_bc = dotProduct(pointToVector(point), edge_bc);

  return ((a_ab < point_ab && point_ab < b_ab) && (b_bc < point_bc && point_bc < c_bc));
}

bool CollisionChecker::checkRectOverlap(const f1tenth_msgs::Rectangle rect_1, const f1tenth_msgs::Rectangle rect_2)
{
  std::vector<geometry_msgs::Vector3> edges = getEdges(rect_1, rect_2);
  std::vector<geometry_msgs::Vector3> axes = getAxes(edges);

  for (auto axis : axes)
  {
    double rect_1_begin, rect_1_end, rect_2_begin, rect_2_end;
    projectRectOnAxis(rect_1, axis, rect_1_begin, rect_1_end);
    projectRectOnAxis(rect_2, axis, rect_2_begin, rect_2_end);

    if (!checkProjectionOverlap(rect_1_begin, rect_1_end, rect_2_begin, rect_2_end))
    {
      return false;
    }
  }

  return true;
}

bool CollisionChecker::checkProjectionOverlap(const double rect_1_min, const double rect_1_max, const double rect_2_min,
                                              const double rect_2_max)
{
  if (rect_1_min < rect_2_min)
  {
    return (rect_2_min - rect_1_max) < 0;
  }
  else
  {
    return (rect_1_min - rect_2_max) < 0;
  }
}

bool CollisionChecker::collisionCheck(const geometry_msgs::PoseStamped vehicle_pose,
                                      const f1tenth_msgs::ObstacleArray obstacles)
{
  collision_index_ = -1;
  computeVehicleFootprint(vehicle_pose);

  for (int i = 0; i < obstacles.obstacles.size(); ++i)
  {
    if (checkRectOverlap(vehicle_footprint_.rectangle, obstacles.obstacles.at(i).footprint.rectangle))
    {
      for (auto point : obstacles.obstacles.at(i).points)
      {
        if (checkPointInRect(point, vehicle_footprint_.rectangle))
        {
          collision_index_ = i;
          return true;
        }
      }
    }
  }

  return false;
}

void CollisionChecker::getCollisionInfo(f1tenth_msgs::RectangleStamped& footprint, int& collision_index)
{
  footprint = vehicle_footprint_;
  collision_index = collision_index_;
}

CollisionChecker::CollisionChecker(const double vehicle_width, const double vehicle_length,
                                   const double base_link_to_center_dist)
  : vehicle_length_(vehicle_length), vehicle_width_(vehicle_width), base_link_to_center_dist_(base_link_to_center_dist)
{
}
}  // namespace safety
}  // namespace f1tenth_racecar
