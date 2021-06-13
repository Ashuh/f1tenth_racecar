#include <algorithm>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "costmap_generator/costmap_generator.h"
#include "costmap_generator/costmap_values.h"

const std::string CostmapGenerator::STATIC_LAYER_ = "static";
const std::string CostmapGenerator::SCAN_LAYER_ = "scan";
const std::string CostmapGenerator::INFLATION_LAYER_ = "inflation";

CostmapGenerator::CostmapGenerator() : tf_listener_(tf_buffer_)
{
  ros::NodeHandle private_nh("~");

  double grid_size_x;
  double grid_size_y;
  double grid_resolution;

  std::string map_topic = "map";
  std::string scan_topic = "scan";
  std::string costmap_topic = "costmap";

  ROS_ASSERT(private_nh.getParam("grid_resolution", grid_resolution));
  ROS_ASSERT(private_nh.getParam("grid_size_x", grid_size_x));
  ROS_ASSERT(private_nh.getParam("grid_size_y", grid_size_y));
  ROS_ASSERT(private_nh.getParam("hard_inflation_radius", hard_inflation_radius_));
  ROS_ASSERT(private_nh.getParam("soft_inflation_radius", soft_inflation_radius_));

  ROS_ASSERT(private_nh.getParam("map_topic", map_topic));
  ROS_ASSERT(private_nh.getParam("scan_topic", scan_topic));
  ROS_ASSERT(private_nh.getParam("costmap_topic", costmap_topic));

  map_sub_ = nh_.subscribe(map_topic, 1, &CostmapGenerator::mapCallback, this);
  scan_sub_ = nh_.subscribe(scan_topic, 1, &CostmapGenerator::scanCallback, this);

  cost_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

  timer_ = nh_.createTimer(ros::Duration(0.1), &CostmapGenerator::timerCallback, this);

  global_map_ = grid_map::GridMap({ STATIC_LAYER_ });
  local_map_ = grid_map::GridMap({ STATIC_LAYER_, SCAN_LAYER_, INFLATION_LAYER_ });
  local_map_.setGeometry(grid_map::Length(grid_size_x, grid_size_y), grid_resolution);
}

void CostmapGenerator::timerCallback(const ros::TimerEvent& timer_event)
{
  local_map_[INFLATION_LAYER_].setConstant(static_cast<int>(CostmapValues::FREE));

  generateStaticLayer();
  inflateOccupiedCells();

  grid_map_msgs::GridMap local_map_msg;
  grid_map::GridMapRosConverter::toMessage(local_map_, local_map_msg);
  cost_map_pub_.publish(local_map_msg);
}

void CostmapGenerator::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  local_map_.setFrameId(scan_msg->header.frame_id);
  local_map_[SCAN_LAYER_].setConstant(static_cast<int>(CostmapValues::FREE));

  for (int i = 0; i < scan_msg->ranges.size(); ++i)
  {
    double theta = scan_msg->angle_min + (scan_msg->angle_increment * i);
    double x = scan_msg->ranges.at(i) * cos(theta);
    double y = scan_msg->ranges.at(i) * sin(theta);
    grid_map::Position pos(x, y);

    if (local_map_.isInside(pos))
    {
      local_map_.atPosition(SCAN_LAYER_, pos) = static_cast<int>(CostmapValues::OCCUPIED);
    }
  }
}

void CostmapGenerator::mapCallback(const nav_msgs::OccupancyGridConstPtr& occ_grid_msg)
{
  ROS_INFO("[Costmap Generator] Received a new map");

  grid_map::GridMapRosConverter::fromOccupancyGrid(*occ_grid_msg, STATIC_LAYER_, global_map_);
}

void CostmapGenerator::generateStaticLayer()
{
  local_map_[STATIC_LAYER_].setConstant(static_cast<int>(CostmapValues::FREE));

  geometry_msgs::TransformStamped transform;

  try
  {
    transform = tf_buffer_.lookupTransform(global_map_.getFrameId(), local_map_.getFrameId(), ros::Time(0));
  }
  catch (const std::exception& ex)
  {
    ROS_ERROR("%s", ex.what());
  }

  grid_map::Matrix& data = local_map_[STATIC_LAYER_];

  for (grid_map::GridMapIterator iterator(local_map_); !iterator.isPastEnd(); ++iterator)
  {
    const grid_map::Index index(*iterator);

    grid_map::Position local_pos;
    local_map_.getPosition(*iterator, local_pos);

    geometry_msgs::PointStamped point;
    point.point.x = local_pos.x();
    point.point.y = local_pos.y();
    tf2::doTransform(point, point, transform);
    grid_map::Position global_pos(point.point.x, point.point.y);

    data(index(0), index(1)) = (global_map_.isInside(global_pos)) ? global_map_.atPosition(STATIC_LAYER_, global_pos) :
                                                                    static_cast<int>(CostmapValues::FREE);
  }
}

void CostmapGenerator::inflateOccupiedCells()
{
  grid_map::Matrix occupancy_data = local_map_[STATIC_LAYER_].cwiseMax(local_map_[SCAN_LAYER_]);

  for (grid_map::GridMapIterator full_it(local_map_); !full_it.isPastEnd(); ++full_it)
  {
    const grid_map::Index center_index(*full_it);

    if (occupancy_data(center_index(0), center_index(1)) == static_cast<int>(CostmapValues::OCCUPIED))
    {
      grid_map::Position center_pos;
      local_map_.getPosition(*full_it, center_pos);
      inflateCell(center_pos);
    }
  }
}

void CostmapGenerator::inflateCell(const grid_map::Position& center_pos)
{
  double scale = soft_inflation_radius_ - hard_inflation_radius_;

  grid_map::Matrix& inflation_data = local_map_[INFLATION_LAYER_];

  for (grid_map::CircleIterator circle_it(local_map_, center_pos, soft_inflation_radius_); !circle_it.isPastEnd();
       ++circle_it)
  {
    const grid_map::Index circle_index(*circle_it);
    double cur_val = inflation_data(circle_index(0), circle_index(1));

    if (cur_val == static_cast<int>(CostmapValues::OCCUPIED))
    {
      continue;
    }

    grid_map::Position it_pos;
    local_map_.getPosition(*circle_it, it_pos);
    double d_x = it_pos.x() - center_pos.x();
    double d_y = it_pos.y() - center_pos.y();
    double dist = sqrt(pow(d_x, 2) + pow(d_y, 2));

    if (dist <= hard_inflation_radius_)
    {
      inflation_data(circle_index(0), circle_index(1)) = static_cast<int>(CostmapValues::OCCUPIED);
    }
    else
    {
      double cost = (soft_inflation_radius_ - dist) / scale * static_cast<int>(CostmapValues::OCCUPIED);
      inflation_data(circle_index(0), circle_index(1)) = std::max(cur_val, cost);
    }
  }
}
