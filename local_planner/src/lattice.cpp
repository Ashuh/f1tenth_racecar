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
#include <visualization_msgs/MarkerArray.h>

#include "costmap_generator/collision_checker.h"
#include "f1tenth_utils/tf2_wrapper.h"
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

Lattice::Generator::Generator(const Pattern& pattern, const double k_length,
                              const std::shared_ptr<CollisionChecker>& collision_checker_ptr)
  : pattern_(pattern)
{
  setLengthWeight(k_length);

  if (collision_checker_ptr != nullptr)
  {
    collision_checker_ptr_ = collision_checker_ptr;
  }
  else
  {
    throw std::invalid_argument("Collision checker pointer is null");
  }
}

Lattice::Generator::Generator(const Generator& generator) : pattern_(generator.pattern_)
{
  k_length_ = generator.k_length_;
  collision_checker_ptr_ = generator.collision_checker_ptr_;
}

Lattice Lattice::Generator::generate(const geometry_msgs::PoseStamped& source_pose) const
{
  // Transform source pose to global path frame
  geometry_msgs::PoseStamped source_pose_transformed =
      TF2Wrapper::doTransform(source_pose, global_path_.header.frame_id);
  std::vector<geometry_msgs::Pose> reference_poses = getReferencePoses(source_pose_transformed.pose);

  Graph graph;
  Vertex source_vertex = generateSourceVertex(source_pose_transformed.pose, reference_poses.at(0));
  VertexDescriptor source_id = boost::add_vertex(source_vertex, graph);
  PositionMap position_map = { { source_vertex.position_, source_id } };
  std::vector<VertexDescriptor> prev_layer_ids{ source_id };

  for (int i = 1; i < reference_poses.size(); ++i)
  {
    std::vector<Position> layer_positions;
    std::vector<VertexDescriptor> layer_ids;
    geometry_msgs::Pose reference_pose = reference_poses.at(i);

    for (int j = -pattern_.num_lateral_samples_per_side_; j <= pattern_.num_lateral_samples_per_side_; ++j)
    {
      Vertex v = generateVertex(reference_pose, i, j);
      VertexDescriptor id = boost::add_vertex(v, graph);
      layer_ids.push_back(id);
      position_map.insert({ v.position_, id });

      for (const auto& prev_id : prev_layer_ids)
      {
        Vertex u = graph[prev_id];

        if (!checkCollision(u, v))
        {
          Edge edge = generateEdge(u, v);
          boost::add_edge(prev_id, id, edge, graph);
        }
      }
    }

    prev_layer_ids = layer_ids;
  }

  return Lattice(graph, position_map, source_vertex.position_, pattern_.num_layers_,
                 2 * pattern_.num_lateral_samples_per_side_ + 1);
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

std::vector<geometry_msgs::Pose> Lattice::Generator::getReferencePoses(const geometry_msgs::Pose& source_pose) const
{
  bool is_loop = global_path_.poses.at(global_path_.poses.size() - 1) == global_path_.poses.at(0);
  int cur_id = getNearestWaypointId(source_pose);
  geometry_msgs::Pose nearest_pose = global_path_.poses.at(cur_id).pose;
  std::vector<geometry_msgs::Pose> poses = { nearest_pose };

  geometry_msgs::Point prev_pos = nearest_pose.position;

  for (int i = 0; i < pattern_.num_layers_; ++i)
  {
    double cur_dist = 0.0;

    while (cur_dist < pattern_.longitudinal_spacing_)
    {
      if (cur_id == global_path_.poses.size())
      {
        if (is_loop)
        {
          cur_id = 0;
        }
        else
        {
          poses.push_back(global_path_.poses.at(global_path_.poses.size() - 1).pose);
          return poses;
        }
      }

      geometry_msgs::Point cur_pos = global_path_.poses.at(cur_id).pose.position;
      cur_dist += distance(cur_pos.x, cur_pos.y, prev_pos.x, prev_pos.y);
      prev_pos = cur_pos;
      ++cur_id;
    }

    poses.push_back(global_path_.poses.at(cur_id - 1).pose);
  }

  return poses;
}

Lattice::Vertex Lattice::Generator::generateSourceVertex(const geometry_msgs::Pose source_pose,
                                                         const geometry_msgs::Pose reference_pose) const
{
  geometry_msgs::Pose relative_pose = TF2Wrapper::doTransform<geometry_msgs::Pose>(source_pose, reference_pose);

  return Vertex(Position(0, relative_pose.position.y / pattern_.lateral_spacing_), source_pose.position.x,
                source_pose.position.y);
}

Lattice::Vertex Lattice::Generator::generateVertex(const geometry_msgs::Pose& reference_pose, const int layer,
                                                   const int lateral_pos) const
{
  double yaw = TF2Wrapper::yawFromQuat(reference_pose.orientation);
  double lateral_offset = lateral_pos * pattern_.lateral_spacing_;
  double x = reference_pose.position.x + lateral_offset * cos(yaw + M_PI_2);
  double y = reference_pose.position.y + lateral_offset * sin(yaw + M_PI_2);

  return Vertex(Position(layer, lateral_pos), x, y);
}

Lattice::Edge Lattice::Generator::generateEdge(const Lattice::Vertex& source, const Lattice::Vertex& target) const
{
  double length = distance(source, target);
  double lateral_distance =
      abs(target.position_.lateral_position_ - source.position_.lateral_position_) * pattern_.lateral_spacing_;

  double weight = k_length_ * length + (1 - k_length_) * lateral_distance;

  return Edge(std::make_shared<Vertex>(source), std::make_shared<Vertex>(target), weight);
}

bool Lattice::Generator::checkCollision(const Lattice::Vertex& source, const Lattice::Vertex& target) const
{
  geometry_msgs::PointStamped source_point;
  geometry_msgs::PointStamped target_point;

  source_point.header.frame_id = global_path_.header.frame_id;
  source_point.point.x = source.x_;
  source_point.point.y = source.y_;

  target_point.header.frame_id = global_path_.header.frame_id;
  target_point.point.x = target.x_;
  target_point.point.y = target.y_;

  return collision_checker_ptr_->checkCollision(source_point, target_point);
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
  collision_checker_ptr_->setCostmap(costmap_msg);
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

void Lattice::Generator::setPattern(const Pattern& pattern)
{
  pattern_ = pattern;
}

/* -------------------------------------------------------------------------- */
/*                          Lattice Generator Pattern                         */
/* -------------------------------------------------------------------------- */

Lattice::Generator::Pattern::Pattern(const int num_layers, const double longitudinal_spacing,
                                     const int num_lateral_samples_per_side, const double lateral_spacing)
{
  if (num_layers <= 0 || longitudinal_spacing <= 0.0 || num_lateral_samples_per_side < 0 || lateral_spacing <= 0.0)
  {
    throw std::invalid_argument("Invalid lattice pattern specified");
  }

  num_layers_ = num_layers;
  longitudinal_spacing_ = longitudinal_spacing;
  num_lateral_samples_per_side_ = num_lateral_samples_per_side;
  lateral_spacing_ = lateral_spacing;
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
    //! BUG
    std::cout << "No path exists" << std::endl;
    return std::vector<geometry_msgs::Point>();
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
    throw std::invalid_argument("Vertex position does not exist in lattice");
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
