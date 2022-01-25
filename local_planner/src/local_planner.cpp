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
#include "f1tenth_utils/tf2_wrapper.h"
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
  initPlanner(private_nh);
  initCallbacks(private_nh);
}

void LocalPlanner::initCallbacks(const ros::NodeHandle& private_nh)
{
  trajectory_pub_ = nh_.advertise<f1tenth_msgs::Trajectory>("local_trajectory", 1);
  viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/local_planner", 1);

  global_path_sub_ = nh_.subscribe("global_path", 1, &LocalPlanner::globalPathCallback, this);
  odom_sub_ = nh_.subscribe("odom", 1, &LocalPlanner::odomCallback, this);
  drive_sub_ = nh_.subscribe("drive_feedback", 1, &LocalPlanner::driveCallback, this);
  timer_ = nh_.createTimer(ros::Duration(0.1), &LocalPlanner::timerCallback, this);

  f_ = boost::bind(&LocalPlanner::configCallback, this, _1, _2);
  server_.setCallback(f_);
}

void LocalPlanner::initPlanner(const ros::NodeHandle& private_nh)
{
  std::vector<double> circle_offsets;

  int lattice_num_layers;
  int lattice_num_lateral_samples_per_side;
  double lattice_layer_spacing;
  double lattice_lateral_spacing;
  double lattice_k_length;

  double ref_max_speed;
  double ref_max_lat_acc;
  double ref_max_lon_acc;
  double ref_max_lon_dec;

  int track_num_paths;
  double max_steering_angle;
  double track_path_lateral_spacing;
  double track_look_ahead_time;
  double track_k_spatial;
  double track_k_temporal;

  nh_.param("wheelbase", wheelbase_, 0.3);
  nh_.param("max_steering_angle", max_steering_angle, 0.4);

  private_nh.param("circle_offsets", circle_offsets, std::vector<double>{ 0.1, 0.3 });

  private_nh.param("lattice_num_layers", lattice_num_layers, 8);
  private_nh.param("lattice_num_lateral_samples_per_side", lattice_num_lateral_samples_per_side, 8);
  private_nh.param("lattice_layer_spacing", lattice_layer_spacing, 2.0);
  private_nh.param("lattice_lateral_spacing", lattice_lateral_spacing, 0.05);
  private_nh.param("lattice_k_length", lattice_k_length, 0.5);
  private_nh.param("ref_max_speed", ref_max_speed, 5.0);
  private_nh.param("ref_max_lat_acc", ref_max_lat_acc, 1.0);
  private_nh.param("ref_max_lon_acc", ref_max_lon_acc, 1.0);
  private_nh.param("ref_max_lon_dec", ref_max_lon_dec, 1.0);

  private_nh.param("track_num_paths", track_num_paths, 9);
  private_nh.param("track_path_lateral_spacing", track_path_lateral_spacing, 0.05);
  private_nh.param("track_look_ahead_time", track_look_ahead_time, 0.5);
  private_nh.param("track_k_spatial", track_k_spatial, 1.0);
  private_nh.param("track_k_temporal", track_k_temporal, 1.0);

  double max_curvature = tan(max_steering_angle) / wheelbase_;

  collision_checker_ptr_ = std::make_shared<CollisionChecker>(circle_offsets);
  viz_ptr_ = std::make_shared<visualization_msgs::MarkerArray>();

  lat_gen_ptr_ = std::make_shared<Lattice::Generator>(lattice_num_layers, lattice_layer_spacing,
                                                      lattice_num_lateral_samples_per_side, lattice_lateral_spacing,
                                                      max_curvature, lattice_k_length, collision_checker_ptr_);

  acc_regulator_ptr_ =
      std::make_shared<AccelerationRegulator>(ref_max_speed, ref_max_lat_acc, ref_max_lon_acc, ref_max_lon_dec);

  ref_traj_gen_ptr_ = std::make_unique<RTG>(lat_gen_ptr_, acc_regulator_ptr_, viz_ptr_);

  double max_lateral_offset = (track_num_paths / 2) * track_path_lateral_spacing;
  trajectory_evaulator_ptr_ = std::make_shared<TrajectoryEvaluator>(collision_checker_ptr_, track_k_spatial,
                                                                    track_k_temporal, max_lateral_offset);

  track_traj_gen_ptr_ = std::make_unique<TTG>(track_num_paths, track_path_lateral_spacing, track_look_ahead_time,
                                              max_curvature, trajectory_evaulator_ptr_, viz_ptr_);
}

void LocalPlanner::timerCallback(const ros::TimerEvent& timer_event)
{
  try
  {
    /* --------------------- Reference Trajectory Generator --------------------- */

    ros::Time ref_begin = ros::Time::now();
    geometry_msgs::PoseStamped current_pose;
    current_pose.header = latest_odom_.header;
    current_pose.pose = latest_odom_.pose.pose;
    current_pose = TF2Wrapper::doTransform<geometry_msgs::PoseStamped>(current_pose, "map");
    Trajectory reference_trajectory = ref_traj_gen_ptr_->generateReferenceTrajectory(current_pose.pose);
    double ref_time = (ros::Time::now() - ref_begin).toSec();

    /* ---------------------- Tracking Trajectory Generator --------------------- */

    ros::Time track_begin = ros::Time::now();
    double current_curvature = tan(current_steering_angle_) / wheelbase_;
    Trajectory tracking_trajectory = track_traj_gen_ptr_->generateTrackingTrajectory(
        reference_trajectory.transform("base_link"), latest_odom_.twist.twist.linear.x, current_curvature);
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
  lat_gen_ptr_->setGlobalPath(path_msg);
}

void LocalPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  latest_odom_ = *odom_msg;
}

void LocalPlanner::driveCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& drive_msg)
{
  current_steering_angle_ = drive_msg->drive.steering_angle;
}

void LocalPlanner::configCallback(local_planner::LocalPlannerConfig& config, uint32_t level)
{
  lat_gen_ptr_->setNumLayers(config.ref_num_layers);
  lat_gen_ptr_->setNumLateralSamplesPerSide(config.ref_num_lateral_samples_per_side);
  lat_gen_ptr_->setLayerSpacing(config.ref_layer_spacing);
  lat_gen_ptr_->setLateralSpacing(config.ref_lateral_spacing);
  lat_gen_ptr_->setMovementWeight(config.ref_k_movement);

  acc_regulator_ptr_->setMaxSpeed(config.ref_speed_limit);
  acc_regulator_ptr_->setMaxLateralAcceleration(config.ref_max_lat_acc);
  acc_regulator_ptr_->setMaxLongitudinalAcceleration(config.ref_max_lon_acc);
  acc_regulator_ptr_->setMaxLongitudinalDeceleration(config.ref_max_lon_dec);

  track_traj_gen_ptr_->setNumPaths(config.track_num_paths);
  track_traj_gen_ptr_->setLateralSpacing(config.track_lateral_spacing);
  track_traj_gen_ptr_->setLookAheadTime(config.track_look_ahead_time);

  trajectory_evaulator_ptr_->setWeights(config.track_k_spatial, config.track_k_temporal);
  trajectory_evaulator_ptr_->setMaxLateralOffset((config.track_num_paths / 2) * config.track_lateral_spacing);
}
