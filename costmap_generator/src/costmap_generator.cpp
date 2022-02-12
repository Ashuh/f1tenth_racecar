#include "costmap_generator/costmap_generator.h"

#include <geometry_msgs/Point.h>
#include <grid_map_msgs/GridMap.h>
#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>

#include <algorithm>
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <string>

#include "costmap_generator/costmap_value.h"
#include "f1tenth_utils/tf2_wrapper.h"

const std::string CostmapGenerator::STATIC_LAYER_ = "STATIC";
const std::string CostmapGenerator::SCAN_LAYER_ = "SCAN";
const std::string CostmapGenerator::INFLATION_LAYER_ = "INFLATION";

CostmapGenerator::CostmapGenerator()
{
  ros::NodeHandle private_nh("~");

  double grid_size_x;
  double grid_size_y;
  double grid_resolution;

  private_nh.param("grid_resolution", grid_resolution, 0.05);
  private_nh.param("grid_size_x", grid_size_x, 10.0);
  private_nh.param("grid_size_y", grid_size_y, 10.0);
  private_nh.param("freespace_distance", freespace_dist_, 0.3);
  private_nh.param("inflation_radius", inflation_radius_, 0.15);

  map_sub_ = nh_.subscribe("map", 1, &CostmapGenerator::mapCallback, this);
  scan_sub_ = nh_.subscribe("scan", 1, &CostmapGenerator::scanCallback, this);
  costmap_pub_ = nh_.advertise<grid_map_msgs::GridMap>("costmap", 1, true);
  timer_ = nh_.createTimer(ros::Duration(0.1), &CostmapGenerator::timerCallback, this);

  global_map_ = grid_map::GridMap({ STATIC_LAYER_ });
  local_map_ = grid_map::GridMap({ STATIC_LAYER_, SCAN_LAYER_ });
  local_map_.setGeometry(grid_map::Length(grid_size_x, grid_size_y), grid_resolution);
}

void CostmapGenerator::timerCallback(const ros::TimerEvent& timer_event)
{
  try
  {
    generateStaticLayer();
    generateInflationLayer();

    // Transform map from vehicle frame to static map frame
    Eigen::Isometry3d tf = tf2::transformToEigen(TF2Wrapper::lookupTransform("map", local_map_.getFrameId()));
    grid_map::GridMap local_map_transformed =
        grid_map::GridMapCvProcessing().getTransformedMap(std::move(local_map_), tf, STATIC_LAYER_, "map");
    grid_map_msgs::GridMap local_map_msg;
    grid_map::GridMapRosConverter::toMessage(local_map_transformed, local_map_msg);
    costmap_pub_.publish(local_map_msg);
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }
}

void CostmapGenerator::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
  local_map_.setFrameId(scan_msg->header.frame_id);
  local_map_[SCAN_LAYER_].setConstant(static_cast<int>(CostmapValue::FREE));

  for (int i = 0; i < scan_msg->ranges.size(); ++i)
  {
    double theta = scan_msg->angle_min + (scan_msg->angle_increment * i);
    double x = scan_msg->ranges.at(i) * cos(theta);
    double y = scan_msg->ranges.at(i) * sin(theta);
    grid_map::Position pos(x, y);

    if (local_map_.isInside(pos))
    {
      local_map_.atPosition(SCAN_LAYER_, pos) = static_cast<int>(CostmapValue::OCCUPIED);
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
  local_map_[STATIC_LAYER_].setConstant(static_cast<int>(CostmapValue::FREE));

  grid_map::Matrix& data = local_map_[STATIC_LAYER_];
  geometry_msgs::TransformStamped tf = TF2Wrapper::lookupTransform(global_map_.getFrameId(), local_map_.getFrameId());

  for (grid_map::GridMapIterator iterator(local_map_); !iterator.isPastEnd(); ++iterator)
  {
    const grid_map::Index index(*iterator);

    grid_map::Position local_pos;
    local_map_.getPosition(*iterator, local_pos);

    geometry_msgs::Point point;
    point.x = local_pos.x();
    point.y = local_pos.y();
    tf2::doTransform(point, point, tf);
    grid_map::Position global_pos(point.x, point.y);

    data(index(0), index(1)) = (global_map_.isInside(global_pos)) ? global_map_.atPosition(STATIC_LAYER_, global_pos) :
                                                                    static_cast<int>(CostmapValue::FREE);
  }
}

void CostmapGenerator::generateInflationLayer()
{
  local_map_.add(INFLATION_LAYER_, local_map_[STATIC_LAYER_].cwiseMax(local_map_[SCAN_LAYER_]));

  cv::Mat image;
  grid_map::GridMapCvConverter::toImage<u_int8_t, 1>(local_map_, INFLATION_LAYER_, CV_8U,
                                                     static_cast<int>(CostmapValue::FREE),
                                                     static_cast<int>(CostmapValue::OCCUPIED), image);
  generateBufferZone(image, image);
  generateConfigurationSpace(image, image);
  grid_map::GridMapCvConverter::addLayerFromImage<u_int8_t, 4>(image, INFLATION_LAYER_, local_map_,
                                                               static_cast<int>(CostmapValue::FREE),
                                                               static_cast<int>(CostmapValue::OCCUPIED));
}

void CostmapGenerator::generateBufferZone(cv::Mat& input, cv::Mat& output)
{
  double max_d = freespace_dist_ / local_map_.getResolution();

  cv::bitwise_not(input, input);
  cv::distanceTransform(input, input, cv::DistanceTypes::DIST_L2, 5, CV_32F);
  cv::threshold(input, input, max_d, 255, cv::THRESH_TRUNC);
  cv::normalize(input, input, 0, 255, cv::NORM_MINMAX);
  input.convertTo(input, CV_8U);
  cv::bitwise_not(input, output);
}

void CostmapGenerator::generateConfigurationSpace(cv::Mat& input, cv::Mat& output)
{
  int kernel_size = 2 * ceil(inflation_radius_ / local_map_.getResolution()) + 1;
  cv::Mat kernel = cv::getStructuringElement(cv::MorphShapes::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size));
  cv::dilate(input, output, kernel);
}
