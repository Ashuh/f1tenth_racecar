#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/imgproc.hpp>

#include "costmap_generator/costmap_layer.h"
#include "costmap_generator/costmap_value.h"
#include "costmap_generator/collision_checker.h"
#include "f1tenth_utils/tf2_wrapper.h"

CollisionChecker::CollisionChecker(const std::vector<double>& circle_offsets, const double circle_radius)
{
  circle_offsets_ = circle_offsets;
  circle_radius_ = circle_radius;
}

bool CollisionChecker::checkCollision(const geometry_msgs::PoseStamped& pose_stamped) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!inflated_costmap_.exists(CostmapLayer::INFLATION))
  {
    throw std::runtime_error("Costmap has not been set for collision checking");
  }

  // Transform input pose to costmap frame
  geometry_msgs::PoseStamped pose_stamped_transformed =
      TF2Wrapper::doTransform(pose_stamped, inflated_costmap_.getFrameId());
  std::vector<geometry_msgs::PointStamped> circle_points = getCirclePositionsFromPose(pose_stamped_transformed);

  for (const auto& point_stamped : circle_points)
  {
    grid_map::Position pos(point_stamped.point.x, point_stamped.point.y);

    if (inflated_costmap_.isInside(pos) &&
        inflated_costmap_.atPosition(CostmapLayer::INFLATION, pos) == static_cast<int>(CostmapValue::OCCUPIED))
    {
      return true;
    }
  }

  return false;
}

bool CollisionChecker::checkCollision(const geometry_msgs::PointStamped& source,
                                      const geometry_msgs::PointStamped& target) const
{
  // Transform points to costmap frame
  geometry_msgs::PointStamped source_transformed = TF2Wrapper::doTransform(source, inflated_costmap_.getFrameId());
  geometry_msgs::PointStamped target_transformed = TF2Wrapper::doTransform(target, inflated_costmap_.getFrameId());
  std::vector<geometry_msgs::PoseStamped> poses = lineToPoses(source_transformed, target_transformed);

  for (auto& pose : poses)
  {
    if (checkCollision(pose))
    {
      return true;
    };
  }

  return false;
}

std::vector<geometry_msgs::PointStamped>
CollisionChecker::getCirclePositionsFromPose(const geometry_msgs::PoseStamped& pose_stamped) const
{
  double yaw = TF2Wrapper::yawFromQuat(pose_stamped.pose.orientation);

  std::vector<geometry_msgs::PointStamped> circle_points;

  for (const auto& offset : circle_offsets_)
  {
    geometry_msgs::PointStamped circle_point;
    circle_point.header.frame_id = pose_stamped.header.frame_id;
    circle_point.point.x = pose_stamped.pose.position.x + offset * cos(yaw);
    circle_point.point.y = pose_stamped.pose.position.y + offset * sin(yaw);
    circle_points.push_back(circle_point);
  }

  return circle_points;
}

std::vector<geometry_msgs::PoseStamped> CollisionChecker::lineToPoses(const geometry_msgs::PointStamped& source,
                                                                      const geometry_msgs::PointStamped& target) const
{
  double x_dist = target.point.x - source.point.x;
  double y_dist = target.point.y - source.point.y;
  geometry_msgs::Quaternion line_orientation = TF2Wrapper::quatFromYaw(atan2(y_dist, x_dist));

  auto minmax_offset = minmax_element(circle_offsets_.begin(), circle_offsets_.end());
  double sampling_interval = *minmax_offset.second - *minmax_offset.first;

  double line_length = sqrt(pow(x_dist, 2) + pow(y_dist, 2));
  double dx = x_dist / line_length * sampling_interval;
  double dy = y_dist / line_length * sampling_interval;

  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = inflated_costmap_.getFrameId();
  pose_stamped.pose.position = source.point;
  pose_stamped.pose.orientation = line_orientation;

  std::vector<geometry_msgs::PoseStamped> poses = { pose_stamped };
  double dist = 0.0;

  while (dist < line_length)
  {
    dist += sampling_interval;

    if (dist > line_length)
    {
      pose_stamped.pose.position = target.point;
    }
    else
    {
      pose_stamped.pose.position.x += dx;
      pose_stamped.pose.position.y += dy;
    }

    poses.push_back(pose_stamped);
  }

  return poses;
}

grid_map::GridMap CollisionChecker::inflateMap(const grid_map::GridMap& costmap, const double radius)
{
  grid_map::GridMap inflated_map({ CostmapLayer::INFLATION });
  inflated_map.setGeometry(costmap.getLength(), costmap.getResolution());
  inflated_map.setFrameId(costmap.getFrameId());
  inflated_map.setPosition(costmap.getPosition());
  inflated_map[CostmapLayer::INFLATION] = costmap[CostmapLayer::STATIC].cwiseMax(costmap[CostmapLayer::SCAN]);

  cv::Mat image;
  grid_map::GridMapCvConverter::toImage<u_int16_t, 1>(inflated_map, CostmapLayer::INFLATION, CV_16U,
                                                      static_cast<int>(CostmapValue::FREE),
                                                      static_cast<int>(CostmapValue::OCCUPIED), image);

  int kernel_size = 2 * ceil(radius / costmap.getResolution()) + 1;
  cv::Mat kernel = cv::getStructuringElement(cv::MorphShapes::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size));
  cv::dilate(image, image, kernel);

  grid_map::GridMapCvConverter::addLayerFromImage<u_int16_t, 4>(image, CostmapLayer::INFLATION, inflated_map,
                                                                static_cast<int>(CostmapValue::FREE),
                                                                static_cast<int>(CostmapValue::OCCUPIED));

  return inflated_map;
}

void CollisionChecker::setCostmap(const grid_map_msgs::GridMap::ConstPtr& costmap_msg)
{
  grid_map::GridMap input_map;
  grid_map::GridMapRosConverter::fromMessage(*costmap_msg, input_map);
  grid_map::GridMap temp = inflateMap(input_map, circle_radius_);

  std::lock_guard<std::mutex> lock(mutex_);
  inflated_costmap_ = temp;
}

nav_msgs::OccupancyGrid CollisionChecker::getInflatedGridMsg() const
{
  nav_msgs::OccupancyGrid grid_msg;
  grid_map::GridMapRosConverter::toOccupancyGrid(inflated_costmap_, CostmapLayer::INFLATION,
                                                 static_cast<int>(CostmapValue::FREE),
                                                 static_cast<int>(CostmapValue::OCCUPIED), grid_msg);
  return grid_msg;
}
