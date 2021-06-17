#ifndef LOCAL_PLANNER_LOCAL_PLANNER_H
#define LOCAL_PLANNER_LOCAL_PLANNER_H

#include <vector>

#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <grid_map_msgs/GridMap.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include "local_planner/path.h"
#include "local_planner/trajectory.h"
#include "local_planner/cubic_spiral_optimizer.h"
#include "local_planner/velocity_profile_generator.h"
#include "local_planner/trajectory_evaluator.h"

class LocalPlanner
{
private:
  ros::NodeHandle nh_;

  ros::Timer timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Subscriber global_path_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber drive_sub_;
  ros::Subscriber costmap_sub_;

  ros::Publisher local_path_pub_;
  ros::Publisher viz_pub_;

  std::unique_ptr<CubicSpiralOptimizer> opt_;
  std::unique_ptr<VelocityProfileGenerator> velocity_gen_;
  std::unique_ptr<TrajectoryEvaluator> traj_eval_;

  nav_msgs::Path global_path_;
  nav_msgs::Odometry latest_odom_;
  double current_steering_angle_;

  double wheelbase_;
  int num_paths_;
  double path_offset_;
  double look_ahead_time_;
  double min_look_ahead_dist_;

  void timerCallback(const ros::TimerEvent& timer_event);

  void globalPathCallback(const nav_msgs::Path& path_msg);

  void odomCallback(const nav_msgs::Odometry& odom_msg);

  void driveCallback(const ackermann_msgs::AckermannDriveStamped& drive_msg);

  void costmapCallback(const grid_map_msgs::GridMap::ConstPtr& costmap_msg);

  void visualizePaths(const std::vector<Trajectory>& trajectories, const std::vector<double>& costs);

  int getNearestWaypointId();

  geometry_msgs::Pose2D getReferenceGoal();

  geometry_msgs::Pose2D generateOffsetGoal(const geometry_msgs::Pose2D& reference_goal, const double lateral_offset);

  // std::vector<geometry_msgs::Pose2D> generateGoals(const geometry_msgs::Pose2D& reference_goal, const int num_goals,
  //                                                  const double lateral_spacing);

  nav_msgs::Path trajectoryToPathMsg(const Trajectory& trajectory);

public:
  LocalPlanner();
};

#endif  // LOCAL_PLANNER_LOCAL_PLANNER_H
