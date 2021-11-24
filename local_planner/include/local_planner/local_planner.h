#ifndef LOCAL_PLANNER_LOCAL_PLANNER_H
#define LOCAL_PLANNER_LOCAL_PLANNER_H

#include <string>
#include <vector>

#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <dynamic_reconfigure/server.h>
#include <grid_map_msgs/GridMap.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

#include "costmap_generator/collision_checker.h"
#include "local_planner/acceleration_regulator.h"
#include "local_planner/reference_trajectory_generator.h"
#include "local_planner/tracking_trajectory_generator.h"
#include "local_planner/trajectory.h"
#include "local_planner/LocalPlannerConfig.h"

class LocalPlanner
{
  using RTG = ReferenceTrajectoryGenerator;
  using TTG = TrackingTrajectoryGenerator;

private:
  ros::NodeHandle nh_;
  ros::Timer timer_;

  ros::Subscriber global_path_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber drive_sub_;

  ros::Publisher trajectory_pub_;
  ros::Publisher viz_pub_;

  dynamic_reconfigure::Server<local_planner::LocalPlannerConfig> server_;
  dynamic_reconfigure::Server<local_planner::LocalPlannerConfig>::CallbackType f_;

  std::unique_ptr<ReferenceTrajectoryGenerator> ref_traj_gen_ptr_;
  std::unique_ptr<TrackingTrajectoryGenerator> track_traj_gen_ptr_;

  std::shared_ptr<Lattice::Generator> lat_gen_ptr_;
  std::shared_ptr<AccelerationRegulator> acc_regulator_ptr_;
  std::shared_ptr<TrajectoryEvaluator> trajectory_evaulator_ptr_;
  std::shared_ptr<CollisionChecker> collision_checker_ptr_;
  std::shared_ptr<visualization_msgs::MarkerArray> viz_ptr_;

  nav_msgs::Odometry latest_odom_;
  double current_steering_angle_;

  double wheelbase_;
  int num_paths_;
  double path_offset_;
  double look_ahead_time_;
  double min_look_ahead_dist_;

  void initCallbacks(const ros::NodeHandle& private_nh);

  void initPlanner(const ros::NodeHandle& private_nh);

  void timerCallback(const ros::TimerEvent& timer_event);

  void globalPathCallback(const nav_msgs::Path::ConstPtr& path_msg);

  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);

  void driveCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& drive_msg);

  void configCallback(local_planner::LocalPlannerConfig& config, uint32_t level);

public:
  LocalPlanner();
};

#endif  // LOCAL_PLANNER_LOCAL_PLANNER_H
