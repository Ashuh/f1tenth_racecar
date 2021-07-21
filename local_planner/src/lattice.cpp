#include <string>
#include <utility>
#include <vector>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>

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
