#include <string>
#include <vector>

#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <dynamic_reconfigure/server.h>
#include <grid_map_msgs/GridMap.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

#include "costmap_generator/collision_checker.h"
#include "f1tenth_msgs/Trajectory.h"
#include "local_planner/lattice.h"
#include "local_planner/acceleration_regulator.h"
#include "local_planner/reference_trajectory_generator.h"
#include "local_planner/tracking_trajectory_generator.h"
#include "local_planner/trajectory.h"
#include "local_planner/trajectory_evaluator.h"
#include "local_planner/local_planner.h"
#include "local_planner/LocalPlannerConfig.h"

LocalPlanner::LocalPlanner()
{
  ros::NodeHandle private_nh("~");
  initCallbacks(private_nh);
  initShared(private_nh);
  initRTG(private_nh);
  initTTG(private_nh);

  /* --------------------------- Dynamic Reconfigure -------------------------- */

  f_ = boost::bind(&LocalPlanner::configCallback, this, _1, _2);
  server_.setCallback(f_);
}

void LocalPlanner::initCallbacks(const ros::NodeHandle& private_nh)
{
  std::string costmap_topic;
  std::string global_path_topic;
  std::string local_path_topic;
  std::string odom_topic;
  std::string inflated_costmap_topic;
  std::string viz_topic;
  std::string drive_topic;

  private_nh.param("costmap_topic", costmap_topic, std::string("costmap"));
  private_nh.param("global_path_topic", global_path_topic, std::string("path/global"));
  private_nh.param("local_path_topic", local_path_topic, std::string("path/local"));
  private_nh.param("odom_topic", odom_topic, std::string("odom"));
  private_nh.param("drive_topic", drive_topic, std::string("drive"));
  private_nh.param("inflated_costmap_topic", inflated_costmap_topic, std::string("inflated_costmap"));
  private_nh.param("viz_topic", viz_topic, std::string("viz/local_planner"));

  global_path_sub_ = nh_.subscribe(global_path_topic, 1, &LocalPlanner::globalPathCallback, this);
  odom_sub_ = nh_.subscribe(odom_topic, 1, &LocalPlanner::odomCallback, this);
  drive_sub_ = nh_.subscribe(drive_topic, 1, &LocalPlanner::driveCallback, this);
  costmap_sub_ = nh_.subscribe(costmap_topic, 1, &LocalPlanner::costmapCallback, this);
  timer_ = nh_.createTimer(ros::Duration(0.1), &LocalPlanner::timerCallback, this);

  trajectory_pub_ = nh_.advertise<f1tenth_msgs::Trajectory>(local_path_topic, 1);
  viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(viz_topic, 1);
  inflated_costmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(inflated_costmap_topic, 1);
}

void LocalPlanner::initShared(const ros::NodeHandle& private_nh)
{
  std::vector<double> circle_offsets;
  double circle_radius;

  private_nh.param("circle_offsets", circle_offsets, std::vector<double>{ 0.1, 0.3 });
  private_nh.param("circle_radius", circle_radius, 0.2);

  collision_checker_ptr_ = std::make_shared<CollisionChecker>(circle_offsets, circle_radius);
  viz_ptr_ = std::make_shared<visualization_msgs::MarkerArray>();
}

void LocalPlanner::initRTG(const ros::NodeHandle& private_nh)
{
  int lattice_num_layers;
  int lattice_num_lateral_samples_per_side;
  double lattice_longitudinal_spacing;
  double lattice_lateral_spacing;
  double lattice_k_length;

  double ref_max_speed;
  double ref_max_lat_acc;
  double ref_max_lon_acc;
  double ref_max_lon_dec;

  private_nh.param("lattice_num_layers", lattice_num_layers, 8);
  private_nh.param("lattice_num_lateral_samples_per_side", lattice_num_lateral_samples_per_side, 8);
  private_nh.param("lattice_longitudinal_spacing", lattice_longitudinal_spacing, 2.0);
  private_nh.param("lattice_lateral_spacing", lattice_lateral_spacing, 0.05);
  private_nh.param("lattice_k_length", lattice_k_length, 0.5);

  private_nh.param("ref_max_speed", ref_max_speed, 5.0);
  private_nh.param("ref_max_lat_acc", ref_max_lat_acc, 1.0);
  private_nh.param("ref_max_lon_acc", ref_max_lon_acc, 1.0);
  private_nh.param("ref_max_lon_dec", ref_max_lon_dec, 1.0);

  Lattice::Generator::Pattern lattice_pattern(lattice_num_layers, lattice_longitudinal_spacing,
                                              lattice_num_lateral_samples_per_side, lattice_lateral_spacing);
  Lattice::Generator lat_gen(lattice_pattern, lattice_k_length, collision_checker_ptr_);
  AccelerationRegulator::Constraints ref_vel_constraints(ref_max_speed, ref_max_lat_acc, ref_max_lon_acc,
                                                         ref_max_lon_dec);

  ref_traj_gen_ptr_ = std::make_unique<RTG>(lat_gen, ref_vel_constraints, viz_ptr_);
}

void LocalPlanner::initTTG(const ros::NodeHandle& private_nh)
{
  int track_num_paths;
  double max_steering_angle;
  double track_path_lateral_spacing;
  double track_look_ahead_time;
  double track_k_spatial;
  double track_k_temporal;

  private_nh.param("track_num_paths", track_num_paths, 9);
  private_nh.param("wheelbase", wheelbase_, 0.3);
  private_nh.param("max_steering_angle", max_steering_angle, 0.4);
  private_nh.param("track_path_lateral_spacing", track_path_lateral_spacing, 0.05);
  private_nh.param("track_look_ahead_time", track_look_ahead_time, 0.5);
  private_nh.param("track_k_spatial", track_k_spatial, 1.0);
  private_nh.param("track_k_temporal", track_k_temporal, 1.0);

  trajectory_evaulator_ptr_ = std::make_shared<TrajectoryEvaluator>(track_k_spatial, track_k_temporal);
  TTG::SamplingPattern tt_pattern(track_num_paths, track_path_lateral_spacing, track_look_ahead_time);
  double max_curvature = tan(max_steering_angle) / wheelbase_;
  track_traj_gen_ptr_ =
      std::make_unique<TTG>(tt_pattern, max_curvature, collision_checker_ptr_, trajectory_evaulator_ptr_, viz_ptr_);
}

void LocalPlanner::timerCallback(const ros::TimerEvent& timer_event)
{
  try
  {
    /* --------------------- Reference Trajectory Generator --------------------- */

    ros::Time ref_begin = ros::Time::now();
    Trajectory reference_trajectory = ref_traj_gen_ptr_->generateReferenceTrajectory(latest_odom_.pose.pose);
    double ref_time = (ros::Time::now() - ref_begin).toSec();

    /* ---------------------- Tracking Trajectory Generator --------------------- */

    ros::Time track_begin = ros::Time::now();
    double current_curvature = tan(current_steering_angle_) / wheelbase_;
    Trajectory tracking_trajectory = track_traj_gen_ptr_->generateTrackingTrajectory(
        reference_trajectory, latest_odom_.twist.twist.linear.x, current_curvature);
    double track_time = (ros::Time::now() - track_begin).toSec();

    ROS_INFO("[Local Planner] Time: [Total] %.3fs [Ref] %.3fs [Track] %.3fs", ref_time + track_time, ref_time,
             track_time);

    /* --------------------------- Publish Trajectory --------------------------- */

    trajectory_pub_.publish(tracking_trajectory.transform("map").toMsg());
  }
  catch (const std::exception& ex)
  {
    ROS_ERROR("[Local Planner] %s", ex.what());
  }

  viz_pub_.publish(*viz_ptr_);
  viz_ptr_->markers.clear();
}

void LocalPlanner::globalPathCallback(const nav_msgs::Path::ConstPtr& path_msg)
{
  ref_traj_gen_ptr_->setGlobalPath(path_msg);
}

void LocalPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  latest_odom_ = *odom_msg;
}

void LocalPlanner::driveCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& drive_msg)
{
  current_steering_angle_ = drive_msg->drive.steering_angle;
}

void LocalPlanner::costmapCallback(const grid_map_msgs::GridMap::ConstPtr& costmap_msg)
{
  ros::Time begin = ros::Time::now();

  collision_checker_ptr_->setCostmap(costmap_msg);
  inflated_costmap_pub_.publish(collision_checker_ptr_->getInflatedGridMsg());

  ROS_INFO("[Local Planner] CostmapCallback completed in %.3fs", (ros::Time::now() - begin).toSec());
}

void LocalPlanner::configCallback(local_planner::LocalPlannerConfig& config, uint32_t level)
{
  ref_traj_gen_ptr_->setLengthWeight(config.ref_k_length);

  Lattice::Generator::Pattern lat_gen_pattern(config.ref_num_layers, config.ref_longitudinal_spacing,
                                              config.ref_num_lateral_samples_per_side, config.ref_lateral_spacing);
  ref_traj_gen_ptr_->setLatticePattern(lat_gen_pattern);

  AccelerationRegulator::Constraints ref_vel_constraints(config.ref_speed_limit, config.ref_max_lat_acc,
                                                         config.ref_max_lon_acc, config.ref_max_lon_dec);
  ref_traj_gen_ptr_->setVelocityConstraints(ref_vel_constraints);

  TTG::SamplingPattern track_sampling_pattern(config.track_num_paths, config.track_lateral_spacing,
                                              config.track_look_ahead_time);
  track_traj_gen_ptr_->setSamplingPattern(track_sampling_pattern);

  trajectory_evaulator_ptr_->setWeights(config.track_k_spatial, config.track_k_temporal);
}
