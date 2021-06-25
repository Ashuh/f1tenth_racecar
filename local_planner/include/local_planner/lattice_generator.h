#ifndef LOCAL_PLANNER_LATTICE_GENERATOR_H
#define LOCAL_PLANNER_LATTICE_GENERATOR_H

#include <vector>

#include <nav_msgs/Path.h>

#include "local_planner/lattice.h"

class LatticeGenerator
{
private:
  int num_layers_;
  double longitudinal_spacing_;

  int num_lateral_samples_;
  double lateral_spacing_;

  double k_length_;

  nav_msgs::Path global_path_;

  std::vector<int> getReferenceWaypointIds(const int nearest_wp_id) const;

  int getLayerWaypointId(const int nearest_wp_id, const int layer) const;

  std::vector<std::vector<Lattice::Vertex>> generateLayers(const std::vector<int>& ref_waypoint_ids,
                                                           const double source_x, const double source_y) const;

  Lattice::Vertex generateVertexAtLayer(const std::vector<int> layer_waypoint_ids, const int layer,
                                        const int lateral_pos) const;

  Lattice::Edge generateEdge(Lattice::Vertex source, Lattice::Vertex target) const;

  double distance(const double x_1, const double y_1, const double x_2, const double y_2) const;

  double distance(const Lattice::Vertex& source, Lattice::Vertex& target) const;

public:
  LatticeGenerator(const int num_layers, const double longitudinal_spacing, const int num_lateral_samples,
                   const double lateral_spacing);

  Lattice generateLattice(const int nearest_wp_id, const double source_x, const double source_y) const;

  void setGlobalPath(const nav_msgs::Path& global_path);

  void setLengthWeight(const double weight);
};

#endif  // LOCAL_PLANNER_LATTICE_GENERATOR_H
