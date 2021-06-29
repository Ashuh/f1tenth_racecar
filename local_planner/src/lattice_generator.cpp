#include <vector>

#include <boost/graph/adjacency_list.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/Path.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "costmap_generator/costmap_layer.h"
#include "costmap_generator/costmap_value.h"
#include "local_planner/lattice_generator.h"

LatticeGenerator::LatticeGenerator(const int num_layers, const double longitudinal_spacing,
                                   const int num_lateral_samples_per_side, const double lateral_spacing,
                                   const double k_length)
  : tf_listener_(tf_buffer_)
{
  num_layers_ = num_layers;
  longitudinal_spacing_ = longitudinal_spacing;
  num_lateral_samples_per_side_ = num_lateral_samples_per_side;
  lateral_spacing_ = lateral_spacing;

  if (k_length < 0.0 || k_length > 1.0)
  {
    throw std::invalid_argument("k_length must be between 0 and 1");
  }
  else
  {
    k_length_ = k_length;
  }
}

Lattice LatticeGenerator::generateLattice(const int nearest_wp_id, const double source_x, const double source_y) const
{
  std::vector<int> ref_waypoint_ids = getReferenceWaypointIds(nearest_wp_id);
  std::vector<std::vector<Lattice::Vertex>> layers = generateLayers(ref_waypoint_ids, source_x, source_y);

  Lattice::Graph graph;
  Lattice::PositionMap position_map;

  for (int i = 0; i < layers.size() - 1; ++i)
  {
    std::vector<Lattice::Vertex> cur_layer = layers.at(i);
    std::vector<Lattice::Vertex> next_layer = layers.at(i + 1);

    for (int j = 0; j < cur_layer.size(); ++j)
    {
      Lattice::Vertex source_vertex = cur_layer.at(j);
      Lattice::VertexDescriptor source_id;

      auto src_it = position_map.find(source_vertex.position_);

      if (src_it == position_map.end())
      {
        source_id = boost::add_vertex(source_vertex, graph);
        position_map.insert({ source_vertex.position_, source_id });
      }
      else
      {
        source_id = src_it->second;
      }

      for (int k = 0; k < next_layer.size(); k++)
      {
        Lattice::Vertex target_vertex = next_layer.at(k);
        Lattice::VertexDescriptor target_id;

        auto tgt_it = position_map.find(target_vertex.position_);

        if (tgt_it == position_map.end())
        {
          target_id = boost::add_vertex(graph);
          position_map.insert({ target_vertex.position_, target_id });
        }
        else
        {
          target_id = tgt_it->second;
        }

        graph[source_id] = source_vertex;
        graph[target_id] = target_vertex;

        if (!checkCollision(source_vertex, target_vertex))
        {
          Lattice::Edge edge = generateEdge(source_vertex, target_vertex);
          boost::add_edge(source_id, target_id, edge, graph);
        }
      }
    }
  }

  Lattice::Position source_position = layers.at(0).at(0).position_;

  return Lattice(graph, position_map, source_position, num_layers_, 2 * num_lateral_samples_per_side_ + 1);
}

std::vector<int> LatticeGenerator::getReferenceWaypointIds(const int nearest_wp_id) const
{
  std::vector<int> waypoint_ids;

  for (int i = 0; i < num_layers_; ++i)
  {
    int layer_wp_id = getLayerWaypointId(nearest_wp_id, i);
    waypoint_ids.push_back(layer_wp_id);
  }

  return waypoint_ids;
}

int LatticeGenerator::getLayerWaypointId(const int nearest_wp_id, const int layer) const
{
  const double layer_distance = layer * longitudinal_spacing_;

  double prev_x = global_path_.poses.at(nearest_wp_id).pose.position.x;
  double prev_y = global_path_.poses.at(nearest_wp_id).pose.position.y;
  double cur_dist = 0.0;

  for (int i = nearest_wp_id + 1; i < global_path_.poses.size(); ++i)
  {
    double cur_x = global_path_.poses.at(i).pose.position.x;
    double cur_y = global_path_.poses.at(i).pose.position.y;
    cur_dist += distance(cur_x, cur_y, prev_x, prev_y);

    if (cur_dist >= layer_distance)
    {
      return i;
    }

    prev_x = cur_x;
    prev_y = cur_y;
  }

  return global_path_.poses.size() - 1;
}

std::vector<std::vector<Lattice::Vertex>> LatticeGenerator::generateLayers(const std::vector<int>& ref_waypoint_ids,
                                                                           const double source_x,
                                                                           const double source_y) const
{
  std::vector<std::vector<Lattice::Vertex>> layers;

  // Generate source vertex
  double nearest_x = global_path_.poses.at(ref_waypoint_ids.at((0))).pose.position.x;
  double nearest_y = global_path_.poses.at(ref_waypoint_ids.at((0))).pose.position.y;

  geometry_msgs::Pose nearest_wp_pose = global_path_.poses.at(ref_waypoint_ids.at((0))).pose;
  geometry_msgs::Pose origin_pose;
  origin_pose.orientation.w = 1;

  tf2::Transform nearest_wp_tf;
  tf2::Transform origin_tf;

  tf2::fromMsg(nearest_wp_pose, nearest_wp_tf);
  tf2::fromMsg(origin_pose, origin_tf);

  geometry_msgs::TransformStamped transform;
  transform.transform = tf2::toMsg(nearest_wp_tf.inverseTimes(origin_tf));

  geometry_msgs::Point source_point;
  source_point.x = source_x;
  source_point.y = source_y;

  tf2::doTransform(source_point, source_point, transform);
  int offset_pos = source_point.y / lateral_spacing_;

  Lattice::Vertex source_vertex(Lattice::Position(0, offset_pos), source_x, source_y);
  std::vector<Lattice::Vertex> source_layer;
  source_layer.push_back(source_vertex);
  layers.push_back(source_layer);

  // Generate regular layers of vertices
  for (int i = 1; i < ref_waypoint_ids.size(); ++i)
  {
    std::vector<Lattice::Vertex> layer;

    for (int j = -num_lateral_samples_per_side_; j <= num_lateral_samples_per_side_; ++j)
    {
      Lattice::Vertex vertex = generateVertexAtLayer(ref_waypoint_ids, i, j);
      layer.push_back(vertex);
    }

    layers.push_back(layer);
  }

  return layers;
}

Lattice::Vertex LatticeGenerator::generateVertexAtLayer(const std::vector<int> layer_waypoint_ids, const int layer,
                                                        const int lateral_pos) const
{
  tf2::Quaternion quat_tf;
  tf2::fromMsg(global_path_.poses.at(layer_waypoint_ids.at(layer)).pose.orientation, quat_tf);
  double dummy;
  double yaw;
  tf2::Matrix3x3(quat_tf).getEulerYPR(yaw, dummy, dummy);

  double ref_x = global_path_.poses.at(layer_waypoint_ids.at(layer)).pose.position.x;
  double ref_y = global_path_.poses.at(layer_waypoint_ids.at(layer)).pose.position.y;
  double ref_yaw = yaw;

  double lateral_offset = lateral_pos * lateral_spacing_;
  double x_offset = lateral_offset * cos(ref_yaw + M_PI_2);
  double y_offset = lateral_offset * sin(ref_yaw + M_PI_2);

  return Lattice::Vertex(Lattice::Position(layer, lateral_pos), ref_x + x_offset, ref_y + y_offset);
}

Lattice::Edge LatticeGenerator::generateEdge(const Lattice::Vertex& source, const Lattice::Vertex& target) const
{
  double length = distance(source, target);
  double lateral_distance =
      abs(target.position_.lateral_position_ - source.position_.lateral_position_) * lateral_spacing_;

  double weight = k_length_ * length + (1 - k_length_) * lateral_distance;

  return Lattice::Edge(weight);
}

bool LatticeGenerator::checkCollision(const Lattice::Vertex& source, const Lattice::Vertex& target) const
{
  if (!costmap_.exists(CostmapLayer::INFLATION))
  {
    throw std::runtime_error(CostmapLayer::INFLATION + " layer is not available in costmap");
  }

  geometry_msgs::TransformStamped transform;

  try
  {
    transform = tf_buffer_.lookupTransform(costmap_.getFrameId(), "map", ros::Time(0));
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_ERROR("[Local Planner] %s", ex.what());
    return true;
  }

  geometry_msgs::Point source_point;
  source_point.x = source.x_;
  source_point.y = source.y_;
  tf2::doTransform(source_point, source_point, transform);

  geometry_msgs::Point target_point;
  target_point.x = target.x_;
  target_point.y = target.y_;
  tf2::doTransform(target_point, target_point, transform);

  grid_map::Position start_pos(source_point.x, source_point.y);
  grid_map::Position end_pos(target_point.x, target_point.y);

  if (!costmap_.isInside(start_pos) || !costmap_.isInside(end_pos))
  {
    // Assume no collision if edge is outside of costmap
    return false;
  }

  const grid_map::Matrix& data = costmap_[CostmapLayer::INFLATION];
  for (grid_map::LineIterator iterator(costmap_, start_pos, end_pos); !iterator.isPastEnd(); ++iterator)
  {
    const grid_map::Index index(*iterator);

    if (data(index(0), index(1)) == static_cast<int>(CostmapValue::OCCUPIED))
    {
      return true;
    }
  }

  return false;
}

double LatticeGenerator::distance(const double x_1, const double y_1, const double x_2, const double y_2) const
{
  double d_x = x_1 - x_2;
  double d_y = y_1 - y_2;
  return sqrt(pow(d_x, 2) + pow(d_y, 2));
}

double LatticeGenerator::distance(const Lattice::Vertex& source, const Lattice::Vertex& target) const
{
  return distance(source.x_, source.y_, target.x_, target.y_);
}

void LatticeGenerator::setGlobalPath(const nav_msgs::Path& global_path)
{
  global_path_ = global_path;
}

void LatticeGenerator::setCostmap(const grid_map_msgs::GridMap::ConstPtr& costmap_msg)
{
  grid_map::GridMapRosConverter::fromMessage(*costmap_msg, costmap_);
}

void LatticeGenerator::setLengthWeight(const double weight)
{
  if (weight >= 0.0 && weight <= 1.0)
  {
    k_length_ = weight;
  }
  else
  {
    throw std::invalid_argument("Weight must be between 0 and 1");
  }
}

void LatticeGenerator::setNumLayers(const int num_layers)
{
  if (num_layers > 1)
  {
    num_layers_ = num_layers;
  }
}

void LatticeGenerator::setNumLateralSamplesPerSide(const int num_samples_per_side)
{
  if (num_samples_per_side >= 0)
  {
    num_lateral_samples_per_side_ = num_samples_per_side;
  }
}

void LatticeGenerator::setLongitudinalSpacing(const double spacing)
{
  if (spacing > 0.0)
  {
    longitudinal_spacing_ = spacing;
  }
}

void LatticeGenerator::setLateralSpacing(const double spacing)
{
  if (spacing > 0.0)
  {
    lateral_spacing_ = spacing;
  }
}
