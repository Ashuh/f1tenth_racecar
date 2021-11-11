#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/optional.hpp>
#include <geometry_msgs/Point.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

#include "costmap_generator/collision_checker.h"
#include "f1tenth_utils/math.h"
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
/*                                   Vertex                                   */
/* -------------------------------------------------------------------------- */

Lattice::Vertex::Vertex()
{
}

Lattice::Vertex::Vertex(const double x, const double y, const double yaw, const double lateral_offset)
{
  x_ = x;
  y_ = y;
  yaw_ = yaw;
  lateral_offset_ = lateral_offset;
}

bool Lattice::Vertex::isOnSameSide(const Lattice::Vertex& v) const
{
  return lateral_offset_ * v.lateral_offset_ >= 0;
}

double Lattice::Vertex::distanceTo(const Vertex& v) const
{
  return calculateDistance(x_, y_, v.x_, v.y_);
}

geometry_msgs::Point Lattice::Vertex::getPoint() const
{
  geometry_msgs::Point point;
  point.x = x_;
  point.y = y_;

  return point;
}

geometry_msgs::Pose Lattice::Vertex::getPose() const
{
  geometry_msgs::Pose pose;
  pose.position = getPoint();
  pose.orientation = TF2Wrapper::quatFromYaw(yaw_);

  return pose;
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
/*                              Lattice Generator                             */
/* -------------------------------------------------------------------------- */

Lattice::Generator::Generator(const int num_layers, const double layer_spacing, const int num_lateral_samples_per_side,
                              const double lateral_spacing, const double max_curvature, const double k_movement,
                              const std::shared_ptr<CollisionChecker>& collision_checker_ptr)
{
  setNumLayers(num_layers);
  setNumLateralSamplesPerSide(num_lateral_samples_per_side);
  setLayerSpacing(layer_spacing);
  setLateralSpacing(lateral_spacing);
  setMaxCurvature(max_curvature);
  setMovementWeight(k_movement);

  if (collision_checker_ptr == nullptr)
  {
    throw std::invalid_argument("Collision checker cannot be nullptr");
  }

  collision_checker_ptr_ = collision_checker_ptr;
}
Lattice Lattice::Generator::generate(const geometry_msgs::Pose& source_pose) const
{
  std::vector<geometry_msgs::Pose> reference_poses = getReferencePoses(source_pose);
  PositionMap map(reference_poses.size());

  Graph graph;
  double source_offset = TF2Wrapper::doTransform<geometry_msgs::Pose>(source_pose, reference_poses.at(0)).position.y;
  Vertex source_vertex(source_pose.position.x, source_pose.position.y, TF2Wrapper::yawFromQuat(source_pose.orientation),
                       source_offset);
  VertexDescriptor source_id = boost::add_vertex(source_vertex, graph);
  map.at(0).insert({ 0, source_id });

  for (int i = 1; i < reference_poses.size(); ++i)
  {
    geometry_msgs::Pose reference_pose = reference_poses.at(i);

    for (int j = -num_lateral_samples_per_side_; j <= num_lateral_samples_per_side_; ++j)
    {
      Vertex v = generateVertex(reference_pose, i, j);
      VertexDescriptor id = boost::add_vertex(v, graph);
      map.at(i).insert({ j, id });

      for (const auto& pair : map.at(i - 1))
      {
        auto prev_id = pair.second;
        Vertex u = graph[prev_id];

        if (calculateCurvature(u.getPose(), v.getPose().position) < max_curvature_ && !checkCollision(u, v))
        {
          double lateral_movement = v.lateral_offset_ - u.lateral_offset_;

          double cumulative_offset;

          if (u.isOnSameSide(v))
          {
            cumulative_offset = std::abs(0.5 * (u.lateral_offset_ + v.lateral_offset_));
          }
          else
          {
            double intercept = -u.lateral_offset_ / (v.lateral_offset_ - u.lateral_offset_);
            cumulative_offset =
                0.5 * (intercept * std::abs(u.lateral_offset_) + (1 - intercept) * std::abs(v.lateral_offset_));
          }

          double weight = k_movement_ * pow(lateral_movement, 2) + (1 - k_movement_) * cumulative_offset;

          if (!v.isOnSameSide(source_vertex))
          {
            weight *= 1.5;  // temp value
          }

          Edge e(weight);
          boost::add_edge(prev_id, id, e, graph);
        }
      }
    }
  }

  return Lattice(graph, map, source_id, num_layers_, 2 * num_lateral_samples_per_side_ + 1);
}

int Lattice::Generator::getNearestWaypointId(const geometry_msgs::Pose& current_pose) const
{
  if (global_path_->poses.empty())
  {
    throw std::runtime_error("Global path has not been set");
  }

  int nearest_wp_id = -1;
  double nearest_wp_dist = std::numeric_limits<double>::max();

  for (int i = 0; i < global_path_->poses.size(); ++i)
  {
    double dist =
        calculateDistance(current_pose.position.x, current_pose.position.y, global_path_->poses.at(i).pose.position.x,
                          global_path_->poses.at(i).pose.position.y);

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
  bool is_loop = global_path_->poses.at(global_path_->poses.size() - 1) == global_path_->poses.at(0);
  int cur_id = getNearestWaypointId(source_pose);
  geometry_msgs::Pose nearest_pose = global_path_->poses.at(cur_id).pose;
  std::vector<geometry_msgs::Pose> poses = { nearest_pose };

  geometry_msgs::Point prev_pos = nearest_pose.position;

  for (int i = 0; i < num_layers_ - 1; ++i)
  {
    double cur_dist = 0.0;

    while (cur_dist < layer_spacing_)
    {
      if (cur_id == global_path_->poses.size())
      {
        if (is_loop)
        {
          cur_id = 0;
        }
        else
        {
          poses.push_back(global_path_->poses.at(global_path_->poses.size() - 1).pose);
          return poses;
        }
      }

      geometry_msgs::Point cur_pos = global_path_->poses.at(cur_id).pose.position;
      cur_dist += calculateDistance(cur_pos.x, cur_pos.y, prev_pos.x, prev_pos.y);
      prev_pos = cur_pos;
      ++cur_id;
    }

    poses.push_back(global_path_->poses.at(cur_id - 1).pose);
  }

  return poses;
}

Lattice::Vertex Lattice::Generator::generateSourceVertex(const geometry_msgs::Pose source_pose,
                                                         const geometry_msgs::Pose reference_pose) const
{
  geometry_msgs::Pose relative_pose = TF2Wrapper::doTransform<geometry_msgs::Pose>(source_pose, reference_pose);

  return Vertex(source_pose.position.x, source_pose.position.y, TF2Wrapper::yawFromQuat(source_pose.orientation),
                relative_pose.position.y);
}

Lattice::Vertex Lattice::Generator::generateVertex(const geometry_msgs::Pose& reference_pose, const int layer,
                                                   const int lateral_pos) const
{
  double yaw = TF2Wrapper::yawFromQuat(reference_pose.orientation);
  double offset = lateral_pos * lateral_spacing_;
  double x = reference_pose.position.x + offset * cos(yaw + M_PI_2);
  double y = reference_pose.position.y + offset * sin(yaw + M_PI_2);

  return Vertex(x, y, TF2Wrapper::yawFromQuat(reference_pose.orientation), offset);
}

bool Lattice::Generator::checkCollision(const Lattice::Vertex& source, const Lattice::Vertex& target) const
{
  geometry_msgs::PointStamped source_point;
  geometry_msgs::PointStamped target_point;

  source_point.header.frame_id = global_path_->header.frame_id;
  source_point.point.x = source.x_;
  source_point.point.y = source.y_;

  target_point.header.frame_id = global_path_->header.frame_id;
  target_point.point.x = target.x_;
  target_point.point.y = target.y_;

  return collision_checker_ptr_->checkCollision(source_point, target_point);
}

void Lattice::Generator::setGlobalPath(const nav_msgs::PathConstPtr& path)
{
  global_path_ = path;
}

void Lattice::Generator::setCostmap(const grid_map_msgs::GridMap::ConstPtr& costmap_msg)
{
  collision_checker_ptr_->setCostmap(costmap_msg);
}

void Lattice::Generator::setNumLayers(const int num_layers)
{
  if (num_layers < 4)
  {
    throw std::invalid_argument("Number of layers must be at least 4");
  }

  layer_spacing_ = num_layers_ = num_layers;
}

void Lattice::Generator::setNumLateralSamplesPerSide(const int num_lateral_samples_per_side)
{
  if (num_lateral_samples_per_side < 0)
  {
    throw std::invalid_argument("Number of lateral samples per side must be non-negative");
  }

  num_lateral_samples_per_side_ = num_lateral_samples_per_side;
}

void Lattice::Generator::setLayerSpacing(const double layer_spacing)
{
  if (layer_spacing <= 0.0)
  {
    throw std::invalid_argument("Layer spacing must be positive");
  }

  layer_spacing_ = layer_spacing;
}

void Lattice::Generator::setLateralSpacing(const double lateral_spacing)
{
  if (lateral_spacing <= 0.0)
  {
    throw std::invalid_argument("Lateral spacing must be positive");
  }

  lateral_spacing_ = lateral_spacing;
}

void Lattice::Generator::setMaxCurvature(const double curvature)
{
  if (curvature <= 0.0)
  {
    throw std::invalid_argument("Maximum curvature must be positive");
  }

  max_curvature_ = curvature;
}

void Lattice::Generator::setMovementWeight(const double weight)
{
  if (weight < 0.0 || weight > 1.0)
  {
    throw std::invalid_argument("Weight must be between 0 and 1");
  }

  k_movement_ = weight;
}

/* -------------------------------------------------------------------------- */
/*                                   Lattice                                  */
/* -------------------------------------------------------------------------- */

Lattice::Lattice(const Graph& graph, const PositionMap& position_map, const VertexDescriptor source_id,
                 const int num_layers, const int num_lateral_samples)
{
  graph_ = graph;
  position_map_ = position_map;
  source_id_ = source_id;
  num_layers_ = num_layers;
  num_lateral_samples_ = num_lateral_samples;
}

void Lattice::computeShortestPaths()
{
  distances_.reserve(boost::num_vertices(graph_));
  predecessors_.reserve(boost::num_vertices(graph_));

  boost::dijkstra_shortest_paths(
      graph_, source_id_,
      boost::weight_map(boost::get(&Edge::weight_, graph_))
          .distance_map(boost::make_iterator_property_map(distances_.begin(), boost::get(boost::vertex_index, graph_)))
          .predecessor_map(
              boost::make_iterator_property_map(predecessors_.begin(), boost::get(boost::vertex_index, graph_))));
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

boost::optional<std::pair<std::vector<geometry_msgs::Point>, double>> Lattice::getShortestPath(const int layer,
                                                                                               const int offset) const
{
  VertexDescriptor goal_id = getVertexIdFromPosition({ layer, offset });

  if (predecessors_[goal_id] == goal_id)
  {
    return boost::none;
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
  Vertex v = graph_[source_id_];
  geometry_msgs::Point point;
  point.x = v.x_;
  point.y = v.y_;
  path.push_back(point);

  std::reverse(path.begin(), path.end());

  return std::make_pair(path, distances_[goal_id]);
}

Lattice::VertexDescriptor Lattice::getVertexIdFromPosition(const Position& pos) const
{
  if (pos.layer_ >= position_map_.size())
  {
    throw std::invalid_argument("Vertex position does not exist in lattice");
  }

  auto it = position_map_.at(pos.layer_).find(pos.lateral_position_);

  if (it == position_map_.at(pos.layer_).end())
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

  auto vertices = boost::vertices(graph_);

  for (auto it = vertices.first; it != vertices.second; ++it)
  {
    vertex_marker.points.push_back(graph_[*it].getPoint());
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

  auto edges = boost::edges(graph_);

  for (auto it = edges.first; it != edges.second; ++it)
  {
    edge_marker.points.push_back(graph_[boost::source(*it, graph_)].getPoint());
    edge_marker.points.push_back(graph_[boost::target(*it, graph_)].getPoint());
  }

  return edge_marker;
}

visualization_msgs::MarkerArray Lattice::generateWeightMarkers(int marker_id, const std::string& ns, const double scale,
                                                               const double r, const double g, const double b,
                                                               const double a) const
{
  visualization_msgs::MarkerArray weight_markers;
  auto edges = boost::edges(graph_);

  for (auto it = edges.first; it != edges.second; ++it)
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
    oss << std::fixed << std::setprecision(2) << graph_[*it].weight_;
    text_marker.text = oss.str();

    auto u = graph_[boost::source(*it, graph_)];
    auto v = graph_[boost::target(*it, graph_)];

    geometry_msgs::Point text_point;
    text_point.x = u.x_ + (v.x_ - u.x_) / 8;
    text_point.y = u.y_ + (v.y_ - u.y_) / 8;
    text_point.z = 0.1;

    text_marker.pose.position = text_point;
    weight_markers.markers.push_back(text_marker);
  }

  return weight_markers;
}
