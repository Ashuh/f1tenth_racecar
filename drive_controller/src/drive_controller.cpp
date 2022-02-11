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

  private_nh.getParam("look_ahead_dist", look_ahead_dist);
  private_nh.getParam("gain", gain);

  trajectory_sub_ = nh_.subscribe("trajectory", 1, &DriveController::trajectoryCallback, this);
  odom_sub_ = nh_.subscribe("odom", 1, &DriveController::odomCallback, this);
  drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>("drive_auto", 1);
  viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/drive_controller", 1);
  timer_ = nh_.createTimer(ros::Duration(0.1), &DriveController::timerCallback, this);

  viz_ptr_ = std::make_shared<visualization_msgs::MarkerArray>();
  pure_pursuit_ = std::make_unique<PurePursuit>(look_ahead_dist, gain, viz_ptr_);

  f_ = boost::bind(&DriveController::configCallback, this, _1, _2);
  server_.setCallback(f_);
}

void DriveController::timerCallback(const ros::TimerEvent& timer_event)
{
  try
  {
    geometry_msgs::Pose cur_pose =
        TF2Wrapper::doTransform(odom_msg_.pose.pose, trajectrory_.header.frame_id, odom_msg_.header.frame_id);
    ackermann_msgs::AckermannDriveStamped drive_msg =
        pure_pursuit_->computeDrive(cur_pose, trajectrory_, odom_msg_.child_frame_id);
    ROS_INFO_STREAM("[Drive Controller] Steering Angle: " << drive_msg.drive.steering_angle * 180.0 / M_PI
                                                          << " degrees");
    drive_pub_.publish(drive_msg);
    viz_pub_.publish(*viz_ptr_);
    viz_ptr_->markers.clear();
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
