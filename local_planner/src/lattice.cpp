#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <geometry_msgs/Point.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/Path.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>

#include "costmap_generator/costmap_layer.h"
#include "costmap_generator/costmap_value.h"
#include "local_planner/lattice.h"

/* -------------------------------------------------------------------------- */
/*                              Lattice Position                              */
/* -------------------------------------------------------------------------- */

Lattice::Position::Position()
{
  layer_ = 0;
  lateral_position_ = 0;
}

Lattice::Position::Position(int layer, int lateral_position)
{
  layer_ = layer;
  lateral_position_ = lateral_position;
}

bool Lattice::Position::operator==(const Position& other) const
{
  return (layer_ == other.layer_ && lateral_position_ == other.lateral_position_);
}

/* -------------------------------------------------------------------------- */
/*                            Lattice Position Hash                           */
/* -------------------------------------------------------------------------- */

size_t Lattice::Position::Hash::operator()(const Lattice::Position& p) const
{
  return p.layer_ * p.lateral_position_;
}

/* -------------------------------------------------------------------------- */
/*                                    Edge                                    */
/* -------------------------------------------------------------------------- */

Lattice::Edge::Edge()
{
}

Lattice::Edge::Edge(const double weight)
{
  weight_ = weight;
}

/* -------------------------------------------------------------------------- */
/*                                   Vertex                                   */
/* -------------------------------------------------------------------------- */

Lattice::Vertex::Vertex()
{
}

Lattice::Vertex::Vertex(const Lattice::Position& position, const double x, const double y)
{
  position_ = position;
  x_ = x;
  y_ = y;
}

/* -------------------------------------------------------------------------- */
/*                              Lattice Generator                             */
/* -------------------------------------------------------------------------- */

Lattice::Generator::Generator(const int num_layers, const double longitudinal_spacing,
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

Lattice Lattice::Generator::generateLattice(const geometry_msgs::Pose& source_pose) const
{
  int nearest_wp_id = getNearestWaypointId(source_pose);
  std::vector<int> ref_waypoint_ids = getReferenceWaypointIds(nearest_wp_id);
  std::vector<std::vector<Lattice::Vertex>> layers =
      generateLayers(ref_waypoint_ids, source_pose.position.x, source_pose.position.y);

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

int Lattice::Generator::getNearestWaypointId(const geometry_msgs::Pose& current_pose) const
{
  if (global_path_.poses.empty())
  {
    throw std::runtime_error("Global path has not been set");
  }

  int nearest_wp_id = -1;
  double nearest_wp_dist = std::numeric_limits<double>::max();

  for (int i = 0; i < global_path_.poses.size(); ++i)
  {
    double dist = distance(current_pose.position.x, current_pose.position.y, global_path_.poses.at(i).pose.position.x,
                           global_path_.poses.at(i).pose.position.y);

    if (dist < nearest_wp_dist)
    {
      nearest_wp_id = i;
      nearest_wp_dist = dist;
    }
  }

  return nearest_wp_id;
}

std::vector<int> Lattice::Generator::getReferenceWaypointIds(const int nearest_wp_id) const
{
  std::vector<int> waypoint_ids;

  for (int i = 0; i < num_layers_; ++i)
  {
    int layer_wp_id = getLayerWaypointId(nearest_wp_id, i);
    waypoint_ids.push_back(layer_wp_id);
  }

  return waypoint_ids;
}

int Lattice::Generator::getLayerWaypointId(const int nearest_wp_id, const int layer) const
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

std::vector<std::vector<Lattice::Vertex>> Lattice::Generator::generateLayers(const std::vector<int>& ref_waypoint_ids,
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

Lattice::Vertex Lattice::Generator::generateVertexAtLayer(const std::vector<int> layer_waypoint_ids, const int layer,
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

Lattice::Edge Lattice::Generator::generateEdge(const Lattice::Vertex& source, const Lattice::Vertex& target) const
{
  double length = distance(source, target);
  double lateral_distance =
      abs(target.position_.lateral_position_ - source.position_.lateral_position_) * lateral_spacing_;

  double weight = k_length_ * length + (1 - k_length_) * lateral_distance;

  return Lattice::Edge(weight);
}

bool Lattice::Generator::checkCollision(const Lattice::Vertex& source, const Lattice::Vertex& target) const
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

double Lattice::Generator::distance(const double x_1, const double y_1, const double x_2, const double y_2) const
{
  double d_x = x_1 - x_2;
  double d_y = y_1 - y_2;
  return sqrt(pow(d_x, 2) + pow(d_y, 2));
}

double Lattice::Generator::distance(const Lattice::Vertex& source, const Lattice::Vertex& target) const
{
  return distance(source.x_, source.y_, target.x_, target.y_);
}

void Lattice::Generator::setGlobalPath(const nav_msgs::Path& global_path)
{
  global_path_ = global_path;
}

void Lattice::Generator::setCostmap(const grid_map_msgs::GridMap::ConstPtr& costmap_msg)
{
  grid_map::GridMapRosConverter::fromMessage(*costmap_msg, costmap_);
}

void Lattice::Generator::setLengthWeight(const double weight)
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

void Lattice::Generator::setNumLayers(const int num_layers)
{
  if (num_layers > 1)
  {
    num_layers_ = num_layers;
  }
}

void Lattice::Generator::setNumLateralSamplesPerSide(const int num_samples_per_side)
{
  if (num_samples_per_side >= 0)
  {
    num_lateral_samples_per_side_ = num_samples_per_side;
  }
}

void Lattice::Generator::setLongitudinalSpacing(const double spacing)
{
  if (spacing > 0.0)
  {
    longitudinal_spacing_ = spacing;
  }
}

void Lattice::Generator::setLateralSpacing(const double spacing)
{
  if (spacing > 0.0)
  {
    lateral_spacing_ = spacing;
  }
}

/* -------------------------------------------------------------------------- */
/*                                   Lattice                                  */
/* -------------------------------------------------------------------------- */

Lattice::Lattice(const Graph& graph, const PositionMap& position_map, const Position& source_position,
                 const int num_layers, const int num_lateral_samples)
{
  graph_ = graph;
  position_map_ = position_map;
  source_position_ = source_position;
  num_layers_ = num_layers;
  num_lateral_samples_ = num_lateral_samples;
  predecessors_ = computeShortestPathsPredecessors();
}

std::vector<Lattice::VertexDescriptor> Lattice::computeShortestPathsPredecessors() const
{
  VertexDescriptor source_id = position_map_.find(source_position_)->second;

  std::vector<double> distances(boost::num_vertices(graph_));  // might not need this, leaving here for now
  std::vector<VertexDescriptor> predecessors(boost::num_vertices(graph_));

  boost::dijkstra_shortest_paths(
      graph_, source_id,
      boost::weight_map(boost::get(&Edge::weight_, graph_))
          .distance_map(boost::make_iterator_property_map(distances.begin(), boost::get(boost::vertex_index, graph_)))
          .predecessor_map(
              boost::make_iterator_property_map(predecessors.begin(), boost::get(boost::vertex_index, graph_))));

  return predecessors;
}

std::vector<Lattice::Vertex> Lattice::getVertices() const
{
  std::vector<Lattice::Vertex> vertices;

  auto vs = boost::vertices(graph_);
  for (auto vit = vs.first; vit != vs.second; ++vit)
  {
    Vertex v = graph_[*vit];
    vertices.push_back(v);
  }

  return vertices;
}

void Lattice::getConnectedVertexPairs(std::vector<std::pair<Vertex, Vertex>>& vertex_pairs,
                                      std::vector<Lattice::Edge>& edges) const
{
  std::vector<std::pair<Vertex, Vertex>> pairs;

  auto es = boost::edges(graph_);

  for (auto eit = es.first; eit != es.second; ++eit)
  {
    edges.push_back(graph_[*eit]);
    Vertex source = graph_[boost::source(*eit, graph_)];
    Vertex target = graph_[boost::target(*eit, graph_)];
    vertex_pairs.push_back(std::pair<Vertex, Vertex>(source, target));
  }
}

std::vector<geometry_msgs::Point> Lattice::getShortestPath(const int offset_pos) const
{
  VertexDescriptor source_id = position_map_.find(source_position_)->second;
  VertexDescriptor goal_id;

  try
  {
    goal_id = getVertexIdFromPosition(Position(num_layers_ - 1, offset_pos));
  }
  catch (const std::invalid_argument& ex)
  {
    // Goal position does not exist in lattice
    return std::vector<geometry_msgs::Point>{};
  }

  if (predecessors_[goal_id] == goal_id)
  {
    std::cout << "No path exists" << std::endl;
    return std::vector<geometry_msgs::Point>{};
  }

  std::vector<geometry_msgs::Point> path;

  for (VertexDescriptor current = goal_id; current != predecessors_[current]; current = predecessors_[current])
  {
    Vertex v = graph_[current];
    geometry_msgs::Point point;
    point.x = v.x_;
    point.y = v.y_;
    path.push_back(point);
  }
  Vertex v = graph_[source_id];
  geometry_msgs::Point point;
  point.x = v.x_;
  point.y = v.y_;
  path.push_back(point);

  return path;
}

Lattice::VertexDescriptor Lattice::getVertexIdFromPosition(const Position& pos) const
{
  auto it = position_map_.find(pos);

  if (it == position_map_.end())
  {
    throw std::invalid_argument("Goal vertex does not exist in lattice");
  }

  return it->second;
}

int Lattice::getNumLayers() const
{
  return num_layers_;
}

int Lattice::getNumLateralSamples() const
{
  return num_lateral_samples_;
}

visualization_msgs::Marker Lattice::generateVertexMarker(const int marker_id, const std::string& ns, const double scale,
                                                         const double r, const double g, const double b,
                                                         const double a) const
{
  auto vertices = getVertices();

  visualization_msgs::Marker vertex_marker;
  vertex_marker.header.frame_id = "map";
  vertex_marker.action = visualization_msgs::Marker::ADD;
  vertex_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  vertex_marker.ns = ns;
  vertex_marker.id = marker_id;
  vertex_marker.lifetime = ros::Duration(0.1);
  vertex_marker.scale.x = scale;
  vertex_marker.scale.y = scale;
  vertex_marker.scale.z = scale;
  vertex_marker.color.r = r;
  vertex_marker.color.g = g;
  vertex_marker.color.b = b;
  vertex_marker.color.a = a;

  for (auto& v : vertices)
  {
    geometry_msgs::Point point;
    point.x = v.x_;
    point.y = v.y_;
    vertex_marker.points.push_back(point);
  }

  return vertex_marker;
}

visualization_msgs::Marker Lattice::generateEdgeMarker(const int marker_id, const std::string& ns, const double scale,
                                                       const double r, const double g, const double b,
                                                       const double a) const
{
  std::vector<Lattice::Edge> edges;
  std::vector<std::pair<Lattice::Vertex, Lattice::Vertex>> vertex_pairs;
  getConnectedVertexPairs(vertex_pairs, edges);

  visualization_msgs::Marker edge_marker;
  edge_marker.header.frame_id = "map";
  edge_marker.action = visualization_msgs::Marker::ADD;
  edge_marker.type = visualization_msgs::Marker::LINE_LIST;
  edge_marker.ns = ns;
  edge_marker.id = marker_id;
  edge_marker.lifetime = ros::Duration(0.1);
  edge_marker.scale.x = scale;

  edge_marker.color.r = r;
  edge_marker.color.g = g;
  edge_marker.color.b = b;
  edge_marker.color.a = a;

  for (auto& pair : vertex_pairs)
  {
    geometry_msgs::Point source;
    geometry_msgs::Point target;

    source.x = pair.first.x_;
    source.y = pair.first.y_;
    target.x = pair.second.x_;
    target.y = pair.second.y_;

    edge_marker.points.push_back(source);
    edge_marker.points.push_back(target);
  }

  return edge_marker;
}

visualization_msgs::MarkerArray Lattice::generateWeightMarkers(int marker_id, const std::string& ns, const double scale,
                                                               const double r, const double g, const double b,
                                                               const double a) const
{
  std::vector<Lattice::Edge> edges;
  std::vector<std::pair<Lattice::Vertex, Lattice::Vertex>> vertex_pairs;
  getConnectedVertexPairs(vertex_pairs, edges);

  visualization_msgs::MarkerArray weight_markers;

  for (int i = 0; i < vertex_pairs.size(); ++i)
  {
    visualization_msgs::Marker text_marker;

    text_marker.header.frame_id = "map";
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.ns = ns;
    text_marker.id = marker_id++;
    text_marker.lifetime = ros::Duration(0.1);
    text_marker.scale.z = scale;
    text_marker.color.r = r;
    text_marker.color.g = g;
    text_marker.color.b = b;
    text_marker.color.a = a;

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2) << edges.at(i).weight_;
    text_marker.text = oss.str();

    geometry_msgs::Point text_point;
    text_point.x = vertex_pairs.at(i).first.x_ + (vertex_pairs.at(i).second.x_ - vertex_pairs.at(i).first.x_) / 8;
    text_point.y = vertex_pairs.at(i).first.y_ + (vertex_pairs.at(i).second.y_ - vertex_pairs.at(i).first.y_) / 8;
    text_point.z = 0.1;

    text_marker.pose.position = text_point;
    weight_markers.markers.push_back(text_marker);
  }

  return weight_markers;
}
