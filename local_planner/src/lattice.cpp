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

#include "costmap_generator/collision_checker.h"
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
/*                                    Edge                                    */
/* -------------------------------------------------------------------------- */

Lattice::Edge::Edge()
{
}

Lattice::Edge::Edge(const std::shared_ptr<Vertex>& source_ptr, const std::shared_ptr<Vertex>& target_ptr,
                    const double weight)
{
  source_ptr_ = source_ptr;
  target_ptr_ = target_ptr;
  weight_ = weight;
}

/* -------------------------------------------------------------------------- */
/*                              Lattice Generator                             */
/* -------------------------------------------------------------------------- */

Lattice::Generator::Generator(const int num_layers, const double longitudinal_spacing,
                              const int num_lateral_samples_per_side, const double lateral_spacing,
                              const double k_length)
  : collision_checker_(std::vector<double>{ 0, 0.2, 0.4 }, 0.3), tf_listener_(tf_buffer_)
{
  setNumLayers(num_layers);
  setLongitudinalSpacing(longitudinal_spacing);
  setNumLateralSamplesPerSide(num_lateral_samples_per_side);
  setLateralSpacing(lateral_spacing);
  setLengthWeight(k_length);
}

Lattice Lattice::Generator::generateLattice(const geometry_msgs::Pose& source_pose) const
{
  int nearest_wp_id = getNearestWaypointId(source_pose);
  std::vector<int> ref_waypoint_ids = getReferenceWaypointIds(nearest_wp_id);
  std::vector<std::vector<Vertex>> layers =
      generateLayers(ref_waypoint_ids, source_pose.position.x, source_pose.position.y);

  Graph graph;
  PositionMap position_map;

  for (int i = 0; i < layers.size() - 1; ++i)
  {
    std::vector<Vertex> cur_layer = layers.at(i);
    std::vector<Vertex> next_layer = layers.at(i + 1);

    for (int j = 0; j < cur_layer.size(); ++j)
    {
      Vertex source_vertex = cur_layer.at(j);
      VertexDescriptor source_id;

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
        Vertex target_vertex = next_layer.at(k);
        VertexDescriptor target_id;

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
          Edge edge = generateEdge(source_vertex, target_vertex);
          boost::add_edge(source_id, target_id, edge, graph);
        }
      }
    }
  }

  Position source_position = layers.at(0).at(0).position_;

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
  std::vector<int> waypoint_ids = { nearest_wp_id };

  for (int i = 0; i < num_layers_; ++i)
  {
    waypoint_ids.push_back(getWaypointIdAtDistance(waypoint_ids.at(i), longitudinal_spacing_));
  }

  return waypoint_ids;
}

int Lattice::Generator::getWaypointIdAtDistance(const int start_id, const int target_dist) const
{
  bool is_loop = global_path_.poses.at(global_path_.poses.size() - 1) == global_path_.poses.at(0);

  double prev_x = global_path_.poses.at(start_id).pose.position.x;
  double prev_y = global_path_.poses.at(start_id).pose.position.y;
  double cur_dist = 0.0;

  int cur_id = start_id;

  while (cur_dist < target_dist)
  {
    if (cur_id == global_path_.poses.size())
    {
      if (is_loop)
      {
        cur_id = 0;
      }
      else
      {
        return global_path_.poses.size() - 1;
      }
    }

    double cur_x = global_path_.poses.at(cur_id).pose.position.x;
    double cur_y = global_path_.poses.at(cur_id).pose.position.y;
    cur_dist += distance(cur_x, cur_y, prev_x, prev_y);

    prev_x = cur_x;
    prev_y = cur_y;
    ++cur_id;
  }

  return cur_id - 1;
}

std::vector<std::vector<Lattice::Vertex>> Lattice::Generator::generateLayers(const std::vector<int>& ref_waypoint_ids,
                                                                             const double source_x,
                                                                             const double source_y) const
{
  std::vector<std::vector<Vertex>> layers;

  // Generate source vertex
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

  Vertex source_vertex(Position(0, offset_pos), source_x, source_y);
  std::vector<Vertex> source_layer;
  source_layer.push_back(source_vertex);
  layers.push_back(source_layer);

  // Generate regular layers of vertices
  for (int i = 1; i < ref_waypoint_ids.size(); ++i)
  {
    std::vector<Vertex> layer;

    for (int j = -num_lateral_samples_per_side_; j <= num_lateral_samples_per_side_; ++j)
    {
      Vertex vertex = generateVertexAtLayer(ref_waypoint_ids, i, j);
      layer.push_back(vertex);
    }

    layers.push_back(layer);
  }

  return layers;
}

Lattice::Vertex Lattice::Generator::generateVertexAtLayer(const std::vector<int>& layer_waypoint_ids, const int layer,
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

  return Vertex(Position(layer, lateral_pos), ref_x + x_offset, ref_y + y_offset);
}

Lattice::Edge Lattice::Generator::generateEdge(const Lattice::Vertex& source, const Lattice::Vertex& target) const
{
  double length = distance(source, target);
  double lateral_distance =
      abs(target.position_.lateral_position_ - source.position_.lateral_position_) * lateral_spacing_;

  double weight = k_length_ * length + (1 - k_length_) * lateral_distance;

  return Edge(std::make_shared<Vertex>(source), std::make_shared<Vertex>(target), weight);
}

bool Lattice::Generator::checkCollision(const Lattice::Vertex& source, const Lattice::Vertex& target) const
{
  geometry_msgs::PointStamped source_point;
  geometry_msgs::PointStamped target_point;

  source_point.header.frame_id = "map";
  source_point.point.x = source.x_;
  source_point.point.y = source.y_;

  target_point.header.frame_id = "map";
  target_point.point.x = target.x_;
  target_point.point.y = target.y_;

  return collision_checker_.checkCollision(source_point, target_point);
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
  collision_checker_.setCostmap(costmap_msg);
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
  if (num_layers > 0)
  {
    num_layers_ = num_layers;
  }
  else
  {
    throw std::invalid_argument("Number of layers must be at least 1");
  }
}

void Lattice::Generator::setNumLateralSamplesPerSide(const int num_samples_per_side)
{
  if (num_samples_per_side >= 0)
  {
    num_lateral_samples_per_side_ = num_samples_per_side;
  }
  else
  {
    throw std::invalid_argument("Number of lateral samples per side cannot be negative");
  }
}

void Lattice::Generator::setLongitudinalSpacing(const double spacing)
{
  if (spacing > 0.0)
  {
    longitudinal_spacing_ = spacing;
  }
  else
  {
    throw std::invalid_argument("Longitudinal spacing must be positive");
  }
}

void Lattice::Generator::setLateralSpacing(const double spacing)
{
  if (spacing > 0.0)
  {
    lateral_spacing_ = spacing;
  }
  else
  {
    throw std::invalid_argument("Lateral spacing must be positive");
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
  std::vector<Vertex> vertices;

  auto vs = boost::vertices(graph_);
  for (auto vit = vs.first; vit != vs.second; ++vit)
  {
    Vertex v = graph_[*vit];
    vertices.push_back(v);
  }

  return vertices;
}

std::vector<Lattice::Edge> Lattice::getEdges() const
{
  std::vector<Edge> edges;

  auto es = boost::edges(graph_);

  for (auto eit = es.first; eit != es.second; ++eit)
  {
    edges.push_back(graph_[*eit]);
  }

  return edges;
}

std::vector<geometry_msgs::Point> Lattice::getShortestPath(const int offset_pos) const
{
  VertexDescriptor source_id = position_map_.find(source_position_)->second;
  VertexDescriptor goal_id;

  try
  {
    goal_id = getVertexIdFromPosition(Position(num_layers_, offset_pos));
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

  for (auto& e : getEdges())
  {
    geometry_msgs::Point source_point;
    geometry_msgs::Point target_point;

    source_point.x = e.source_ptr_->x_;
    source_point.y = e.source_ptr_->y_;
    target_point.x = e.target_ptr_->x_;
    target_point.y = e.target_ptr_->y_;

    edge_marker.points.push_back(source_point);
    edge_marker.points.push_back(target_point);
  }

  return edge_marker;
}

visualization_msgs::MarkerArray Lattice::generateWeightMarkers(int marker_id, const std::string& ns, const double scale,
                                                               const double r, const double g, const double b,
                                                               const double a) const
{
  visualization_msgs::MarkerArray weight_markers;

  for (auto& e : getEdges())
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
    oss << std::fixed << std::setprecision(2) << e.weight_;
    text_marker.text = oss.str();

    geometry_msgs::Point text_point;
    text_point.x = e.source_ptr_->x_ + (e.target_ptr_->x_ - e.source_ptr_->x_) / 8;
    text_point.y = e.source_ptr_->y_ + (e.target_ptr_->y_ - e.source_ptr_->y_) / 8;
    text_point.z = 0.1;

    text_marker.pose.position = text_point;
    weight_markers.markers.push_back(text_marker);
  }

  return weight_markers;
}
