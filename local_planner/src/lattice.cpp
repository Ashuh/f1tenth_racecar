#include <utility>
#include <vector>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

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

Lattice::Lattice(const Graph& graph, const PositionMap& position_map, const int num_layers,
                 const int num_lateral_samples)
{
  graph_ = graph;
  position_map_ = position_map;
  num_layers_ = num_layers;
  num_lateral_samples_ = num_lateral_samples;
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

std::vector<Lattice::Vertex> Lattice::search() const
{
  VertexDescriptor source_id = position_map_.find(Position(0, 0))->second;
  auto goal_it = position_map_.find(Position(num_layers_ - 1, -2));  // temp fixed position

  if (goal_it == position_map_.end())
  {
    throw std::invalid_argument("Goal vertex does not exist in lattice");
  }

  VertexDescriptor goal_id = goal_it->second;

  std::vector<double> distances(boost::num_vertices(graph_));
  std::vector<VertexDescriptor> predecessors(boost::num_vertices(graph_));

  boost::dijkstra_shortest_paths(
      graph_, source_id,
      boost::weight_map(boost::get(&Edge::weight_, graph_))
          .distance_map(boost::make_iterator_property_map(distances.begin(), boost::get(boost::vertex_index, graph_)))
          .predecessor_map(
              boost::make_iterator_property_map(predecessors.begin(), boost::get(boost::vertex_index, graph_))));

  std::vector<Vertex> path;

  for (VertexDescriptor current = goal_id; current != source_id; current = predecessors[current])
  {
    path.push_back(graph_[current]);
  }

  path.push_back(graph_[source_id]);

  return path;
}
