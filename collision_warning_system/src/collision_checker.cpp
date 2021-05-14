#include <limits>
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
geometry_msgs::Polygon CollisionChecker::transformVehicleRect(const geometry_msgs::Pose vehicle_pose)
{
  geometry_msgs::TransformStamped tf;
  tf.transform.translation.x = vehicle_pose.position.x;
  tf.transform.translation.y = vehicle_pose.position.y;
  tf.transform.rotation = vehicle_pose.orientation;

  geometry_msgs::Point front_left_transformed;
  geometry_msgs::Point front_right_transformed;
  geometry_msgs::Point rear_left_transformed;
  geometry_msgs::Point rear_right_transformed;

  tf2::doTransform(front_left_, front_left_transformed, tf);
  tf2::doTransform(front_right_, front_right_transformed, tf);
  tf2::doTransform(rear_left_, rear_left_transformed, tf);
  tf2::doTransform(rear_right_, rear_right_transformed, tf);

  geometry_msgs::Point32 front_left_transformed_32;
  geometry_msgs::Point32 front_right_transformed_32;
  geometry_msgs::Point32 rear_left_transformed_32;
  geometry_msgs::Point32 rear_right_transformed_32;

  front_left_transformed_32.x = front_left_transformed.x;
  front_left_transformed_32.y = front_left_transformed.y;

  front_right_transformed_32.x = front_right_transformed.x;
  front_right_transformed_32.y = front_right_transformed.y;

  rear_left_transformed_32.x = rear_left_transformed.x;
  rear_left_transformed_32.y = rear_left_transformed.y;

  rear_right_transformed_32.x = rear_right_transformed.x;
  rear_right_transformed_32.y = rear_right_transformed.y;

  geometry_msgs::Polygon vehicle_footprint_transformed;

  vehicle_footprint_transformed.points = { front_left_transformed_32, front_right_transformed_32, rear_right_transformed_32,
                                      rear_left_transformed_32 };

  return vehicle_footprint_transformed;
}

geometry_msgs::Vector3 CollisionChecker::pointsToEdge(const geometry_msgs::Point32 p_1,
                                                      const geometry_msgs::Point32 p_2)
{
  geometry_msgs::Vector3 edge;
  edge.x = p_2.x - p_1.x;
  edge.y = p_2.y - p_1.y;

  return edge;
}

geometry_msgs::Vector3 CollisionChecker::pointToVector(const geometry_msgs::Point32 point)
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

std::vector<geometry_msgs::Vector3> CollisionChecker::getEdges(const geometry_msgs::Polygon rect_1,
                                                               const geometry_msgs::Polygon rect_2)
{
  std::vector<geometry_msgs::Vector3> edges;

  edges.push_back(pointsToEdge(rect_1.points.at(0), rect_1.points.at(1)));
  edges.push_back(pointsToEdge(rect_1.points.at(1), rect_1.points.at(2)));

  edges.push_back(pointsToEdge(rect_2.points.at(0), rect_2.points.at(1)));
  edges.push_back(pointsToEdge(rect_2.points.at(1), rect_2.points.at(2)));

  return edges;
}

std::vector<geometry_msgs::Vector3> CollisionChecker::getAxes(const std::vector<geometry_msgs::Vector3> edges)
{
  std::vector<geometry_msgs::Vector3> axes;

  for (auto edge : edges)
  {
    axes.push_back(perpendicular(edge));
  }

  return axes;
}

double CollisionChecker::dotProduct(const geometry_msgs::Vector3 v_1, const geometry_msgs::Vector3 v_2)
{
  return v_1.x * v_2.x + v_1.y * v_2.y;
}

void CollisionChecker::projectRectOnAxis(const geometry_msgs::Polygon rect, const geometry_msgs::Vector3 axis,
                                         double& projection_begin, double& projection_end)
{
  projection_begin = std::numeric_limits<double>::infinity();
  projection_end = -std::numeric_limits<double>::infinity();

  for (auto point : rect.points)
  {
    double projection = dotProduct(pointToVector(point), axis);

    if (projection > projection_end)
    {
      projection_end = projection;
    }

    if (projection < projection_begin)
    {
      projection_begin = projection;
    }
  }
}

bool CollisionChecker::checkPointInRect(const geometry_msgs::Point32 point, const geometry_msgs::Polygon rect)
{
  geometry_msgs::Vector3 edge_ab = pointsToEdge(rect.points.at(0), rect.points.at(1));
  geometry_msgs::Vector3 edge_bc = pointsToEdge(rect.points.at(1), rect.points.at(2));

  geometry_msgs::Vector3 point_vec;
  point_vec.x = point.x;
  point_vec.y = point.y;

  double a_ab = dotProduct(pointToVector(rect.points.at(0)), edge_ab);
  double b_ab = dotProduct(pointToVector(rect.points.at(1)), edge_ab);
  double point_ab = dotProduct(pointToVector(point), edge_ab);

  double b_bc = dotProduct(pointToVector(rect.points.at(1)), edge_bc);
  double c_bc = dotProduct(pointToVector(rect.points.at(2)), edge_bc);
  double point_bc = dotProduct(pointToVector(point), edge_bc);

  return ((a_ab < point_ab && point_ab < b_ab) && (b_bc < point_bc && point_bc < c_bc));
}

bool CollisionChecker::checkRectOverlap(const geometry_msgs::Polygon rect_1, const geometry_msgs::Polygon rect_2)
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

bool CollisionChecker::collisionCheck(const geometry_msgs::Pose vehicle_pose,
                                      const f1tenth_msgs::ObstacleArray obstacles)
{
  vehicle_footprint_.polygon = transformVehicleRect(vehicle_pose);
  vehicle_footprint_.header.frame_id = "laser";

  collision_index_ = -1;

  for (int i = 0; i < obstacles.obstacles.size(); ++i)
  {
    if (checkRectOverlap(vehicle_footprint_.polygon, obstacles.obstacles.at(i).footprint))
    {
      for (int j = 0; j < obstacles.obstacles.at(i).points.size(); j++)
      {
        if (checkPointInRect(obstacles.obstacles.at(i).points.at(j), vehicle_footprint_.polygon))
        {
          collision_index_ = i;
          return true;
        }
      }
    }
  }

  return false;
}

void CollisionChecker::getCollisionInfo(geometry_msgs::PolygonStamped& footprint, int& collision_index)
{
  footprint = vehicle_footprint_;
  collision_index = collision_index_;
}

CollisionChecker::CollisionChecker(const double vehicle_width, const double vehicle_length,
                                   const double rear_axle_from_cg)
{
  front_left_.x = (vehicle_length / 2) + rear_axle_from_cg;
  front_left_.y = -vehicle_width / 2;

  front_right_.x = (vehicle_length / 2) + rear_axle_from_cg;
  front_right_.y = vehicle_width / 2;

  rear_left_.x = (vehicle_length / 2) - rear_axle_from_cg;
  rear_left_.y = -vehicle_width / 2;

  rear_right_.x = (vehicle_length / 2) - rear_axle_from_cg;
  rear_right_.y = vehicle_width / 2;
}
}  // namespace safety
}  // namespace f1tenth_racecar
