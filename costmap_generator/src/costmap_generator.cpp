#include <algorithm>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <tf2_eigen/tf2_eigen.h>

#include "costmap_generator/costmap_generator.h"
#include "costmap_generator/costmap_value.h"
#include "f1tenth_msgs/InflateCostmap.h"
#include "f1tenth_utils/tf2_wrapper.h"

const std::string CostmapGenerator::STATIC_LAYER_ = "STATIC";
const std::string CostmapGenerator::SCAN_LAYER_ = "SCAN";

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
  costmap_pub_ = nh_.advertise<grid_map_msgs::GridMap>(costmap_topic, 1, true);
  timer_ = nh_.createTimer(ros::Duration(0.1), &CostmapGenerator::timerCallback, this);

  global_map_ = grid_map::GridMap({ STATIC_LAYER_ });
  local_map_ = grid_map::GridMap({ STATIC_LAYER_, SCAN_LAYER_ });
  local_map_.setGeometry(grid_map::Length(grid_size_x, grid_size_y), grid_resolution);

  server_ = nh_.advertiseService("inflate_costmap", &CostmapGenerator::serviceCallback, this);
}

void CostmapGenerator::timerCallback(const ros::TimerEvent& timer_event)
{
  try
  {
    generateStaticLayer();

    for (const auto& pair : request_map_)
    {
      double radius = pair.first;
      std::string layer = pair.second.first;
      inflateMap(radius, layer);
    }

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

  kernel_map_.clear();
}

void CostmapGenerator::mapCallback(const nav_msgs::OccupancyGridConstPtr& occ_grid_msg)
{
  ROS_INFO("[Costmap Generator] Received a new map");

  grid_map::GridMapRosConverter::fromOccupancyGrid(*occ_grid_msg, STATIC_LAYER_, global_map_);
  kernel_map_.clear();
}

bool CostmapGenerator::serviceCallback(
    ros::ServiceEvent<f1tenth_msgs::InflateCostmapRequest, f1tenth_msgs::InflateCostmapResponse>& event)
{
  auto request = event.getRequest();
  std::string client_id = event.getCallerName() + "/" + request.client_id;
  std::string layer_id = generateLayerId(request.radius);

  if (event.getRequest().action == f1tenth_msgs::InflateCostmapRequest::ADD)
  {
    ROS_INFO("[Costmap Generator] Received add request for [%.2f m] from [%s]", request.radius, client_id.c_str());

    addRequest(client_id, request.radius, layer_id);
    event.getResponse().layer_id = layer_id;
  }
  else
  {
    ROS_INFO("[Costmap Generator] Received delete request from [%s]", client_id.c_str());

    try
    {
      deleteRequest(client_id);
    }
    catch (const std::invalid_argument& ex)
    {
      ROS_ERROR("[Costmap Generator] %s", ex.what());
      return false;
    }
  }

  ROS_INFO("[Costmap Generator] Request success");
  return true;
}

void CostmapGenerator::addRequest(const std::string& client_id, const double radius, const std::string& layer_id)
{
  std::string layer = generateLayerId(radius);

  bool is_new_client = client_map_.insert({ client_id, radius }).second;

  if (!is_new_client)
  {
    client_map_.at(client_id) = radius;
  }

  auto request_it = request_map_.find(radius);

  if (request_it == request_map_.end())
  {
    ROS_DEBUG("[Costmap Generator] Adding new request [%.2f m]", radius);
    request_map_.insert({ radius, { layer, 1 } });  // create new request with 1 client
  }
  else
  {
    ROS_DEBUG("[Costmap Generator] Request [%.2f m] already exists, incrementing number of clients", radius);
    request_it->second.second++;  // increment number of clients
  }
}

void CostmapGenerator::deleteRequest(const std::string& client_id)
{
  auto client_it = client_map_.find(client_id);

  if (client_it == client_map_.end())
  {
    throw std::invalid_argument(client_id + " does not have an existing request");
  }

  client_map_.erase(client_id);
  double radius = client_it->second;

  ROS_DEBUG("[Costmap Generator] Decrementing number of clients of [%.2f m]", radius);
  int num_remaining_users = --request_map_.find(radius)->second.second;

  if (num_remaining_users == 0)
  {
    ROS_DEBUG("[Costmap Generator] Deleting request [%.2f m] since it has 0 clients", radius);
    request_map_.erase(radius);
  }
}

void CostmapGenerator::generateStaticLayer()
{
  local_map_[STATIC_LAYER_].setConstant(static_cast<int>(CostmapValue::FREE));

  grid_map::Matrix& data = local_map_[STATIC_LAYER_];

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

    data(index(0), index(1)) = (global_map_.isInside(global_pos)) ? global_map_.atPosition(STATIC_LAYER_, global_pos) :
                                                                    static_cast<int>(CostmapValue::FREE);
  }
}

void CostmapGenerator::inflateMap(const double radius, const std::string& layer_id)
{
  int kernel_size = 2 * ceil(radius / local_map_.getResolution()) + 1;
  auto it = kernel_map_.find(kernel_size);

  if (it == kernel_map_.end())
  {
    local_map_.add(layer_id, local_map_[STATIC_LAYER_].cwiseMax(local_map_[SCAN_LAYER_]));

    cv::Mat image;
    grid_map::GridMapCvConverter::toImage<u_int16_t, 1>(local_map_, layer_id, CV_16U,
                                                        static_cast<int>(CostmapValue::FREE),
                                                        static_cast<int>(CostmapValue::OCCUPIED), image);
    kernel_map_.insert({ kernel_size, layer_id });
    cv::Mat kernel = cv::getStructuringElement(cv::MorphShapes::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size));
    cv::dilate(image, image, kernel);
    grid_map::GridMapCvConverter::addLayerFromImage<u_int16_t, 4>(
        image, layer_id, local_map_, static_cast<int>(CostmapValue::FREE), static_cast<int>(CostmapValue::OCCUPIED));
  }
  else
  {
    local_map_.add(layer_id, local_map_[it->second]);
  }
}

std::string CostmapGenerator::generateLayerId(const double radius)
{
  return "inflation/" + std::to_string(radius);
}
