#ifndef LOCAL_PLANNER_LATTICE_H
#define LOCAL_PLANNER_LATTICE_H

#include <string>
#include <utility>
#include <vector>

#include <boost/graph/adjacency_list.hpp>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>

class Lattice
{
public:
  struct Position
  {
    struct Hash
    {
      size_t operator()(const Lattice::Position& p) const;
    };

    int layer_;
    int lateral_position_;

    Position();

    Position(int layer, int lateral_position);

    bool operator==(const Position& other) const;
  };

  struct Edge
  {
    double weight_;

    Edge();

    explicit Edge(const double weight);
  };

  struct Vertex
  {
    Lattice::Position position_;
    double x_;
    double y_;

    Vertex();

    Vertex(const Lattice::Position& position, const double x, const double y);
  };

  typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, Vertex, Edge> Graph;
  typedef Graph::vertex_descriptor VertexDescriptor;
  typedef Graph::edge_descriptor EdgeDescriptor;
  typedef std::unordered_map<Lattice::Position, Lattice::VertexDescriptor, Lattice::Position::Hash> PositionMap;

  Lattice(const Graph& graph, const PositionMap& position_map, const Position& source_position, const int num_layers,
          const int num_lateral_samples);

  std::vector<VertexDescriptor> computeShortestPathsPredecessors() const;

  std::vector<Vertex> getVertices() const;

  void getConnectedVertexPairs(std::vector<std::pair<Vertex, Vertex>>& vertex_pairs,
                               std::vector<Lattice::Edge>& edges) const;

  std::vector<geometry_msgs::Point> getShortestPath(const int offset_pos) const;

  std::vector<geometry_msgs::Point> cubicSplineInterpolate(const std::vector<Vertex>& path) const;

  VertexDescriptor getVertexIdFromPosition(const Position& pos) const;

  int getNumLayers() const;

  int getNumLateralSamples() const;

  visualization_msgs::Marker generateVertexMarker(const int marker_id, const std::string& ns, const double scale,
                                                  const double r, const double g, const double b,
                                                  const double a = 1.0) const;

  visualization_msgs::Marker generateEdgeMarker(const int marker_id, const std::string& ns, const double scale,
                                                const double r, const double g, const double b,
                                                const double a = 1.0) const;

  visualization_msgs::MarkerArray generateWeightMarkers(int marker_id, const std::string& ns, const double scale,
                                                        const double r, const double g, const double b,
                                                        const double a = 1.0) const;

private:
  int num_layers_;
  int num_lateral_samples_;

  Graph graph_;
  Position source_position_;
  PositionMap position_map_;
  std::vector<VertexDescriptor> predecessors_;
};

#endif  // LOCAL_PLANNER_LATTICE_H
