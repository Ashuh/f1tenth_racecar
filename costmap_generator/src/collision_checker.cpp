#include <string>
#include <vector>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>

#include "costmap_generator/costmap_value.h"
#include "costmap_generator/collision_checker.h"
#include "f1tenth_msgs/InflateCostmap.h"
#include "f1tenth_utils/tf2_wrapper.h"

CollisionChecker::CollisionChecker(const std::vector<double>& circle_offsets, const double circle_radius,
                                   const std::string& id)
  : id_(id)
{
  circle_offsets_ = circle_offsets;
  circle_radius_ = circle_radius;

  connect();
  sendInflationRequest(circle_radius);
}

CollisionChecker::~CollisionChecker()
{
  cancelInflationRequest();
}

void CollisionChecker::connect()
{
  client_ = nh_.serviceClient<f1tenth_msgs::InflateCostmap>("inflate_costmap", true);
  client_.waitForExistence();
}

void CollisionChecker::sendInflationRequest(const double radius)
{
  if (!client_.isValid())
  {
    ROS_ERROR("[Collision Checker] Lost connection to service [%s], attempting to reconnect",
              client_.getService().c_str());
    connect();
  }

  f1tenth_msgs::InflateCostmap srv;
  srv.request.client_id = id_;
  srv.request.action = f1tenth_msgs::InflateCostmapRequest::ADD;
  srv.request.radius = circle_radius_;
  client_.call(srv);
  layer_id_ = srv.response.layer_id;
}

void CollisionChecker::cancelInflationRequest()
{
  f1tenth_msgs::InflateCostmap srv;
  srv.request.client_id = id_;
  srv.request.action = f1tenth_msgs::InflateCostmapRequest::DELETE;
  client_.call(srv);
}

bool CollisionChecker::checkCollision(const geometry_msgs::PoseStamped& pose_stamped) const
{
  if (!costmap_.exists(layer_id_))
  {
    throw std::runtime_error("Layer [" + layer_id_ + "] does not exist");
  }

  // Transform input pose to costmap frame
  geometry_msgs::PoseStamped pose_stamped_transformed = TF2Wrapper::doTransform(pose_stamped, costmap_.getFrameId());
  std::vector<geometry_msgs::PointStamped> circle_points = getCirclePositionsFromPose(pose_stamped_transformed);

  for (const auto& point_stamped : circle_points)
  {
    grid_map::Position pos(point_stamped.point.x, point_stamped.point.y);

    if (costmap_.isInside(pos) && costmap_.atPosition(layer_id_, pos) == static_cast<int>(CostmapValue::OCCUPIED))
    {
      return true;
    }
  }

  return false;
}

bool CollisionChecker::checkCollision(const geometry_msgs::PointStamped& source,
                                      const geometry_msgs::PointStamped& target) const
{
  if (!costmap_.exists(layer_id_))
  {
    throw std::runtime_error("Layer [" + layer_id_ + "] does not exist");
  }

  // Transform points to costmap frame
  geometry_msgs::PointStamped source_transformed = TF2Wrapper::doTransform(source, costmap_.getFrameId());
  geometry_msgs::PointStamped target_transformed = TF2Wrapper::doTransform(target, costmap_.getFrameId());
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
  pose_stamped.header.frame_id = costmap_.getFrameId();
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

void CollisionChecker::setCostmap(const grid_map_msgs::GridMap::ConstPtr& costmap_msg)
{
  grid_map::GridMapRosConverter::fromMessage(*costmap_msg, costmap_, { layer_id_ });
}
