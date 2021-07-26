#ifndef LOCAL_PLANNER_LATTICE_H
#define LOCAL_PLANNER_LATTICE_H

#include <string>
#include <utility>
#include <vector>

#include <boost/graph/adjacency_list.hpp>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include "costmap_generator/collision_checker.h"

class Lattice
{
public:
  struct Position
  {
    struct Hash
    {
      size_t operator()(const Position& p) const;
    };

    int layer_;
    int lateral_position_;

    Position();

    Position(int layer, int lateral_position);

    bool operator==(const Position& other) const;
  };

  struct Vertex
  {
    Position position_;
    double x_;
    double y_;

    Vertex();

    Vertex(const Position& position, const double x, const double y);
  };

  struct Edge
  {
    std::shared_ptr<Vertex> source_ptr_;
    std::shared_ptr<Vertex> target_ptr_;

    double weight_;

    Edge();

    Edge(const std::shared_ptr<Vertex>& source_ptr, const std::shared_ptr<Vertex>& target_ptr, const double weight);
  };

  typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, Vertex, Edge> Graph;
  typedef Graph::vertex_descriptor VertexDescriptor;
  typedef Graph::edge_descriptor EdgeDescriptor;
  typedef std::unordered_map<Lattice::Position, Lattice::VertexDescriptor, Lattice::Position::Hash> PositionMap;

  class Generator
  {
  private:
    int num_layers_;
    double longitudinal_spacing_;

    int num_lateral_samples_per_side_;
    double lateral_spacing_;

    double k_length_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    nav_msgs::Path global_path_;
    CollisionChecker collision_checker_;

    int getNearestWaypointId(const geometry_msgs::Pose& current_pose) const;

    std::vector<int> getReferenceWaypointIds(const int nearest_wp_id) const;

    int getWaypointIdAtDistance(const int start_id, const int target_dist) const;

    std::vector<std::vector<Vertex>> generateLayers(const std::vector<int>& ref_waypoint_ids, const double source_x,
                                                    const double source_y) const;

    Vertex generateVertexAtLayer(const std::vector<int>& layer_waypoint_ids, const int layer,
                                 const int lateral_pos) const;

    Edge generateEdge(const Vertex& source, const Vertex& target) const;

    bool checkCollision(const Vertex& source, const Vertex& target) const;

    double distance(const double x_1, const double y_1, const double x_2, const double y_2) const;

    double distance(const Vertex& source, const Vertex& target) const;

  public:
    Generator(const int num_layers, const double longitudinal_spacing, const int num_lateral_samples,
              const double lateral_spacing, const double k_length);

    Lattice generateLattice(const geometry_msgs::Pose& source_pose) const;

    void setGlobalPath(const nav_msgs::Path& global_path);

    void setCostmap(const grid_map_msgs::GridMap::ConstPtr& costmap_msg);

    void setLengthWeight(const double weight);

    void setNumLayers(const int num_layers);

    void setNumLateralSamplesPerSide(const int num_samples_per_side);

    void setLongitudinalSpacing(const double spacing);

    void setLateralSpacing(const double spacing);
  };

  std::vector<VertexDescriptor> computeShortestPathsPredecessors() const;

  std::vector<Vertex> getVertices() const;

  std::vector<Edge> getEdges() const;

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

  Lattice(const Graph& graph, const PositionMap& position_map, const Position& source_position, const int num_layers,
          const int num_lateral_samples);
};

#endif  // LOCAL_PLANNER_LATTICE_H
