#ifndef LOCAL_PLANNER_LOCAL_PLANNER_H
#define LOCAL_PLANNER_LOCAL_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include "local_planner/cubic_spiral_optimizer.h"
#include "local_planner/velocity_profile_generator.h"

class LocalPlanner
{
private:
  ros::NodeHandle nh_;

  ros::Timer timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Subscriber global_path_sub_;
  ros::Subscriber odom_sub_;

  ros::Publisher local_path_pub_;
  ros::Publisher viz_pub_;

  std::unique_ptr<CubicSpiralOptimizer> opt_;
  std::unique_ptr<VelocityProfileGenerator> velocity_gen_;

  nav_msgs::Path global_path_;
  nav_msgs::Odometry latest_odom_;

  int num_paths_;
  double path_offset_;
  double look_ahead_time_;
  double min_look_ahead_dist_;

  void timerCallback(const ros::TimerEvent& timer_event);

  void globalPathCallback(const nav_msgs::Path& path_msg);

  void odomCallback(const nav_msgs::Odometry& odom_msg);

  void visualizePaths(const std::vector<CubicSpiralPath>& paths);

  int getNearestWaypointId();

  geometry_msgs::Pose2D getReferenceGoal();

  std::vector<geometry_msgs::Pose2D> generateGoals(const geometry_msgs::Pose2D& reference_goal, const int num_goals,
                                                   const double lateral_spacing);

public:
  LocalPlanner();
};

#endif  // LOCAL_PLANNER_LOCAL_PLANNER_H
