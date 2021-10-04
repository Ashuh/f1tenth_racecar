#include <algorithm>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <grid_map_cv/GridMapCvProcessing.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <tf2_eigen/tf2_eigen.h>

#include "costmap_generator/costmap_generator.h"
#include "costmap_generator/costmap_layer.h"
#include "costmap_generator/costmap_value.h"
#include "f1tenth_utils/tf2_wrapper.h"

CostmapGenerator::CostmapGenerator()
{
  ros::NodeHandle private_nh("~");

  double grid_size_x;
  double grid_size_y;
  double grid_resolution;

  std::string map_topic;
  std::string scan_topic;
  std::string costmap_topic;

  private_nh.param("grid_resolution", grid_resolution, 0.05);
  private_nh.param("grid_size_x", grid_size_x, 10.0);
  private_nh.param("grid_size_y", grid_size_y, 10.0);

  private_nh.param("map_topic", map_topic, std::string("map"));
  private_nh.param("scan_topic", scan_topic, std::string("scan"));
  private_nh.param("costmap_topic", costmap_topic, std::string("costmap"));

  map_sub_ = nh_.subscribe(map_topic, 1, &CostmapGenerator::mapCallback, this);
  scan_sub_ = nh_.subscribe(scan_topic, 1, &CostmapGenerator::scanCallback, this);
  cost_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>(costmap_topic, 1, true);
  timer_ = nh_.createTimer(ros::Duration(0.1), &CostmapGenerator::timerCallback, this);

  global_map_ = grid_map::GridMap({ CostmapLayer::STATIC });
  local_map_ = grid_map::GridMap({ CostmapLayer::STATIC, CostmapLayer::SCAN, CostmapLayer::INFLATION });
  local_map_.setGeometry(grid_map::Length(grid_size_x, grid_size_y), grid_resolution);
}

void CostmapGenerator::timerCallback(const ros::TimerEvent& timer_event)
{
  local_map_[CostmapLayer::INFLATION].setConstant(static_cast<int>(CostmapValue::FREE));

  generateStaticLayer();

  try
  {
    // Transform map from vehicle frame to static map frame
    Eigen::Isometry3d tf = tf2::transformToEigen(TF2Wrapper::lookupTransform("map", local_map_.getFrameId()));
    grid_map::GridMap local_map_transformed =
        grid_map::GridMapCvProcessing().getTransformedMap(std::move(local_map_), tf, CostmapLayer::STATIC, "map");
    grid_map_msgs::GridMap local_map_msg;
    grid_map::GridMapRosConverter::toMessage(local_map_transformed, local_map_msg);
    cost_map_pub_.publish(local_map_msg);
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
  }
}

void CostmapGenerator::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  local_map_.setFrameId(scan_msg->header.frame_id);
  local_map_[CostmapLayer::SCAN].setConstant(static_cast<int>(CostmapValue::FREE));

  for (int i = 0; i < scan_msg->ranges.size(); ++i)
  {
    double theta = scan_msg->angle_min + (scan_msg->angle_increment * i);
    double x = scan_msg->ranges.at(i) * cos(theta);
    double y = scan_msg->ranges.at(i) * sin(theta);
    grid_map::Position pos(x, y);

    if (local_map_.isInside(pos))
    {
      local_map_.atPosition(CostmapLayer::SCAN, pos) = static_cast<int>(CostmapValue::OCCUPIED);
    }
  }
}

void CostmapGenerator::mapCallback(const nav_msgs::OccupancyGridConstPtr& occ_grid_msg)
{
  ROS_INFO("[Costmap Generator] Received a new map");

  grid_map::GridMapRosConverter::fromOccupancyGrid(*occ_grid_msg, CostmapLayer::STATIC, global_map_);
}

void CostmapGenerator::generateStaticLayer()
{
  local_map_[CostmapLayer::STATIC].setConstant(static_cast<int>(CostmapValue::FREE));
  grid_map::Matrix& data = local_map_[CostmapLayer::STATIC];

  geometry_msgs::TransformStamped transform;

  try
  {
    for (grid_map::GridMapIterator iterator(local_map_); !iterator.isPastEnd(); ++iterator)
    {
      const grid_map::Index index(*iterator);

      grid_map::Position local_pos;
      local_map_.getPosition(*iterator, local_pos);

      geometry_msgs::Point point;
      point.x = local_pos.x();
      point.y = local_pos.y();
      point = TF2Wrapper::doTransform<geometry_msgs::Point>(point, global_map_.getFrameId(), local_map_.getFrameId());
      grid_map::Position global_pos(point.x, point.y);

      data(index(0), index(1)) = (global_map_.isInside(global_pos)) ?
                                     global_map_.atPosition(CostmapLayer::STATIC, global_pos) :
                                     static_cast<int>(CostmapValue::FREE);
    }
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
  }
}

void CostmapGenerator::inflateOccupiedCells()
{
  // grid_map::Matrix occupancy_data = local_map_[CostmapLayer::STATIC].cwiseMax(local_map_[CostmapLayer::SCAN]);

  // for (grid_map::GridMapIterator full_it(local_map_); !full_it.isPastEnd(); ++full_it)
  // {
  //   const grid_map::Index center_index(*full_it);

  //   if (occupancy_data(center_index(0), center_index(1)) == static_cast<int>(CostmapValue::OCCUPIED))
  //   {
  //     grid_map::Position center_pos;
  //     local_map_.getPosition(*full_it, center_pos);
  //     inflateCell(center_pos);
  //   }
  // }
}
