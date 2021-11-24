#ifndef LOCAL_PLANNER_LATTICE_H
#define LOCAL_PLANNER_LATTICE_H

#include <utility>
#include <string>
#include <vector>

#include <boost/graph/adjacency_list.hpp>
#include <boost/optional.hpp>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

#include "costmap_generator/collision_checker.h"

class Lattice
{
private:
  struct Position
  {
    int layer_;
    int lateral_position_;

    Position();

    Position(int layer, int lateral_position);

    bool operator==(const Position& other) const;
  };

  struct Vertex
  {
    double x_;
    double y_;
    double yaw_;
    double lateral_offset_;

    Vertex();

    Vertex(const double x, const double y, const double yaw, const double lateral_offset);

    bool isOnSameSide(const Vertex& v) const;

    double distanceTo(const Vertex& v) const;

    geometry_msgs::Point getPoint() const;

    geometry_msgs::Pose getPose() const;
  };

  struct Edge
  {
    double weight_;

    Edge();

    explicit Edge(const double weight);
  };

  typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, Vertex, Edge> Graph;
  typedef Graph::vertex_descriptor VertexDescriptor;
  typedef Graph::edge_descriptor EdgeDescriptor;
  typedef std::vector<std::unordered_map<int, double>> PositionMap;

  int num_layers_;
  int num_lateral_samples_;

  Graph graph_;
  PositionMap position_map_;
  VertexDescriptor source_id_;
  std::vector<VertexDescriptor> predecessors_;
  std::vector<double> distances_;

  Lattice(const Graph& graph, const PositionMap& position_map, const VertexDescriptor source_id, const int num_layers,
          const int num_lateral_samples);

public:
  class Generator
  {
  public:
    Generator(const int num_layers, const double layer_spacing, const int num_lateral_samples_per_side,
              const double lateral_spacing, double max_curvature, const double k_length,
              const std::shared_ptr<CollisionChecker>& collision_checker_ptr);

    Lattice generate(const geometry_msgs::Pose& source_pose) const;

    void setGlobalPath(const nav_msgs::PathConstPtr& path);

    void setNumLayers(const int num_layers);

    void setNumLateralSamplesPerSide(const int num_lateral_samples_per_side);

    void setLayerSpacing(const double layer_spacing);

    void setLateralSpacing(const double lateral_spacing);

    void setMaxCurvature(const double curvature);

    void setMovementWeight(const double weight);

  private:
    int num_layers_;                    // Number of layers including the source vertex layer
    int num_lateral_samples_per_side_;  // Number of vertices per side in 1ayer
    double layer_spacing_;              // Spacing between layers
    double lateral_spacing_;            // Spacing between vertices in a layer
    double max_curvature_;              // Maximum allowable curvature between 2 vertices for an edge to be added
    double k_movement_;                 // Tradeoff between lateral movement cost and cumulative lateral offset cost

    nav_msgs::PathConstPtr global_path_;

    std::shared_ptr<CollisionChecker> collision_checker_ptr_;

    int getNearestWaypointId(const geometry_msgs::Pose& current_pose) const;

    std::vector<geometry_msgs::Pose> getReferencePoses(const geometry_msgs::Pose& source_pose) const;

    Vertex generateSourceVertex(const geometry_msgs::Pose source_pose, const geometry_msgs::Pose reference_pose) const;

    Vertex generateVertex(const geometry_msgs::Pose& reference_pose, const int layer, const int lateral_pos) const;

    bool checkCollision(const Vertex& source, const Vertex& target) const;
  };

  void computeShortestPaths();

  std::vector<Vertex> getVertices() const;

  boost::optional<std::pair<std::vector<geometry_msgs::Point>, double>> getShortestPath(const int layer,
                                                                                        const int offset) const;

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
};

#endif  // LOCAL_PLANNER_LATTICE_H
