#ifndef LOCAL_PLANNER_LATTICE_GENERATOR_H
#define LOCAL_PLANNER_LATTICE_GENERATOR_H

#include <vector>

#include <geometry_msgs/Pose.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_listener.h>

#include "local_planner/lattice.h"

class LatticeGenerator
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
  grid_map::GridMap costmap_;

  int getNearestWaypointId(const geometry_msgs::Pose& current_pose) const;

  std::vector<int> getReferenceWaypointIds(const int nearest_wp_id) const;

  int getLayerWaypointId(const int nearest_wp_id, const int layer) const;

  std::vector<std::vector<Lattice::Vertex>> generateLayers(const std::vector<int>& ref_waypoint_ids,
                                                           const double source_x, const double source_y) const;

  Lattice::Vertex generateVertexAtLayer(const std::vector<int> layer_waypoint_ids, const int layer,
                                        const int lateral_pos) const;

  Lattice::Edge generateEdge(const Lattice::Vertex& source, const Lattice::Vertex& target) const;

  bool checkCollision(const Lattice::Vertex& source, const Lattice::Vertex& target) const;

  double distance(const double x_1, const double y_1, const double x_2, const double y_2) const;

  double distance(const Lattice::Vertex& source, const Lattice::Vertex& target) const;

public:
  LatticeGenerator(const int num_layers, const double longitudinal_spacing, const int num_lateral_samples,
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

#endif  // LOCAL_PLANNER_LATTICE_GENERATOR_H
