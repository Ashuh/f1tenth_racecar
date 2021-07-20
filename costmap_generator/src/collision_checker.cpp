#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/imgproc.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "costmap_generator/costmap_layer.h"
#include "costmap_generator/costmap_value.h"
#include "costmap_generator/collision_checker.h"

CollisionChecker::CollisionChecker(const std::vector<double>& circle_offsets, const double circle_radius)
  : tf_listener_(tf_buffer_)
{
  circle_offsets_ = circle_offsets;
  circle_radius_ = circle_radius;
}

bool CollisionChecker::checkCollision(const geometry_msgs::PoseStamped& pose_stamped)
{
  if (!inflated_costmap_.exists(CostmapLayer::INFLATION))
  {
    throw std::runtime_error("Costmap has not been set for collision checking");
  }

  geometry_msgs::TransformStamped transform =
      tf_buffer_.lookupTransform(inflated_costmap_.getFrameId(), pose_stamped.header.frame_id, ros::Time(0));

  geometry_msgs::PoseStamped pose_stamped_transformed;
  tf2::doTransform(pose_stamped, pose_stamped_transformed, transform);

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

std::vector<geometry_msgs::PointStamped>
CollisionChecker::getCirclePositionsFromPose(const geometry_msgs::PoseStamped& pose_stamped)
{
  tf2::Quaternion quat_tf;
  tf2::fromMsg(pose_stamped.pose.orientation, quat_tf);
  double dummy, yaw;
  tf2::Matrix3x3(quat_tf).getEulerYPR(yaw, dummy, dummy);

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

grid_map::GridMap CollisionChecker::inflateMap(const grid_map::GridMap& costmap, const double radius)
{
  grid_map::GridMap inflated_map({ CostmapLayer::INFLATION });
  inflated_map.setGeometry(costmap.getLength(), costmap.getResolution());
  inflated_map.setFrameId(costmap.getFrameId());
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
  inflated_costmap_ = inflateMap(input_map, circle_radius_);
}

nav_msgs::OccupancyGrid CollisionChecker::getInflatedGridMsg() const
{
  nav_msgs::OccupancyGrid grid_msg;
  grid_map::GridMapRosConverter::toOccupancyGrid(inflated_costmap_, CostmapLayer::INFLATION,
                                                 static_cast<int>(CostmapValue::FREE),
                                                 static_cast<int>(CostmapValue::OCCUPIED), grid_msg);
  return grid_msg;
}
