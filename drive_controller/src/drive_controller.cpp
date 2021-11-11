#include <string>

#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>

#include "drive_controller/pure_pursuit.h"
#include "drive_controller/drive_controller.h"
#include "f1tenth_msgs/Trajectory.h"
#include "f1tenth_utils/tf2_wrapper.h"

DriveController::DriveController()
{
  ros::NodeHandle private_nh("~");

  double look_ahead_dist;
  double gain;

  std::string trajectory_topic;
  std::string odom_topic;
  std::string drive_topic;
  std::string viz_topic;

  private_nh.getParam("look_ahead_dist", look_ahead_dist);
  private_nh.getParam("gain", gain);
  private_nh.getParam("trajectory_topic", trajectory_topic);
  private_nh.getParam("odom_topic", odom_topic);
  private_nh.getParam("auto_drive_topic", drive_topic);
  private_nh.getParam("viz_topic", viz_topic);

  trajectory_sub_ = nh_.subscribe(trajectory_topic, 1, &DriveController::trajectoryCallback, this);
  odom_sub_ = nh_.subscribe(odom_topic, 1, &DriveController::odomCallback, this);
  drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1);
  viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(viz_topic, 1);
  timer_ = nh_.createTimer(ros::Duration(1.0 / UPDATE_FREQUENCY), &DriveController::timerCallback, this);

  viz_ptr_ = std::make_shared<visualization_msgs::MarkerArray>();
  pure_pursuit_ = std::make_unique<PurePursuit>(look_ahead_dist, gain, viz_ptr_);

  f_ = boost::bind(&DriveController::configCallback, this, _1, _2);
  server_.setCallback(f_);
}

void DriveController::timerCallback(const ros::TimerEvent& timer_event)
{
  if ((ros::Time::now() - trajectrory_.header.stamp).toSec() > 1.0 / UPDATE_FREQUENCY)
  {
    ROS_ERROR("[Drive Controller] Trajectory information is outdated. Publishing empty message.");
    drive_pub_.publish(ackermann_msgs::AckermannDriveStamped());
    return;
  }

  try
  {
    ackermann_msgs::AckermannDriveStamped drive_msg = pure_pursuit_->computeDrive(odom_msg_, trajectrory_);
    ROS_INFO_STREAM("[Drive Controller] Steering Angle: " << drive_msg.drive.steering_angle * 180.0 / M_PI
                                                          << " degrees");
    drive_pub_.publish(drive_msg);
    viz_pub_.publish(*viz_ptr_);
  }
  catch (const std::invalid_argument& ex)
  {
    ROS_ERROR("[Drive Controller] %s", ex.what());
    return;
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_ERROR("[Drive Controller] %s", ex.what());
    return;
  }
}

void DriveController::odomCallback(const nav_msgs::Odometry odom_msg)
{
  odom_msg_ = odom_msg;
}

void DriveController::trajectoryCallback(const f1tenth_msgs::Trajectory traj_msg)
{
  trajectrory_ = traj_msg;
}

void DriveController::configCallback(drive_controller::DriveControllerConfig& config, uint32_t level)
{
  pure_pursuit_->setGain(config.gain);
  pure_pursuit_->setLookAheadDistance(config.look_ahead_distance);
}
