#include <vector>
#include <boost/graph/graph_utility.hpp>

#include <boost/graph/adjacency_list.hpp>
#include <nav_msgs/Path.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "local_planner/lattice_generator.h"

LatticeGenerator::LatticeGenerator(const int num_layers, const double longitudinal_spacing,
                                   const int num_lateral_samples, const double lateral_spacing)
{
  num_layers_ = num_layers;
  longitudinal_spacing_ = longitudinal_spacing;
  num_lateral_samples_ = num_lateral_samples;
  lateral_spacing_ = lateral_spacing;
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

        Lattice::Edge edge = generateEdge(source_vertex, target_vertex);
        boost::add_edge(source_id, target_id, edge, graph);
      }
    }
  }

  //   boost::print_graph(graph);

  return Lattice(graph, position_map, num_layers_, num_lateral_samples_);
}

std::vector<int> LatticeGenerator::getReferenceWaypointIds(const int nearest_wp_id) const
{
  std::vector<int> waypoint_ids;

  for (int i = 0; i < num_layers_; ++i)
  {
    if (i == 0)
    {
      waypoint_ids.push_back(0);  // for source vertex
    }
    else
    {
      int layer_wp_id = getLayerWaypointId(nearest_wp_id, i);
      waypoint_ids.push_back(layer_wp_id);
    }
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
  Lattice::Vertex source_vertex(Lattice::Position(0, 0), source_x, source_y);
  std::vector<Lattice::Vertex> source_layer;
  source_layer.push_back(source_vertex);
  layers.push_back(source_layer);

  // Generate regular layers of vertices
  for (int i = 1; i < ref_waypoint_ids.size(); ++i)
  {
    std::vector<Lattice::Vertex> layer;

    for (int j = -num_lateral_samples_ / 2; j <= num_lateral_samples_ / 2; ++j)
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

Lattice::Edge LatticeGenerator::generateEdge(Lattice::Vertex source, Lattice::Vertex target) const
{
  double length = distance(source, target);
  double lateral_distance =
      abs(target.position_.lateral_position_ - source.position_.lateral_position_) * lateral_spacing_;

  double weight = k_length_ * length + (1 - k_length_) * lateral_distance;

  return Lattice::Edge(weight);
}

double LatticeGenerator::distance(const double x_1, const double y_1, const double x_2, const double y_2) const
{
  double d_x = x_1 - x_2;
  double d_y = y_1 - y_2;
  return sqrt(pow(d_x, 2) + pow(d_y, 2));
}

double LatticeGenerator::distance(const Lattice::Vertex& source, Lattice::Vertex& target) const
{
  return distance(source.x_, source.y_, target.x_, target.y_);
}

void LatticeGenerator::setGlobalPath(const nav_msgs::Path& global_path)
{
  global_path_ = global_path;
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
