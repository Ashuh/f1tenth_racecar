#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>

#include "f1tenth_utils/math.h"
#include "f1tenth_utils/tf2_wrapper.h"

double calculateDistanceSq(const double x1, const double y1, const double x2, const double y2)
{
  return pow(x2 - x1, 2) + pow(y2 - y1, 2);
}

double calculateDistance(const double x1, const double y1, const double x2, const double y2)
{
  return sqrt(calculateDistanceSq(x1, y1, x2, y2));
}

double calculateCurvature(const geometry_msgs::Pose& pose, const geometry_msgs::Point point)
{
  geometry_msgs::Point relative_point = TF2Wrapper::doTransform(point, pose);
  double chord_length_sq = calculateDistanceSq(0, 0, relative_point.x, relative_point.y);

  return 2 * abs(relative_point.y) / chord_length_sq;
}
