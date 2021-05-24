#include <string>

#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

#include "drive_controller/pure_pursuit.h"
#include "drive_controller/drive_controller.h"

namespace f1tenth_racecar
{
namespace control
{
DriveController::DriveController()
{
  ros::NodeHandle private_nh("~");

  double look_ahead_dist;
  double gain;

  std::string path_topic;
  std::string odom_topic;
  std::string drive_topic;
  std::string viz_topic;

  ROS_ASSERT(private_nh.getParam("look_ahead_dist", look_ahead_dist));
  ROS_ASSERT(private_nh.getParam("gain", gain));

  ROS_ASSERT(private_nh.getParam("path_topic", path_topic));
  ROS_ASSERT(private_nh.getParam("odom_topic", odom_topic));
  ROS_ASSERT(private_nh.getParam("drive_topic", drive_topic));
  ROS_ASSERT(private_nh.getParam("viz_topic", viz_topic));

  path_sub_ = nh_.subscribe(path_topic, 1, &DriveController::pathCallback, this);
  odom_sub_ = nh_.subscribe(odom_topic, 1, &DriveController::odomCallback, this);

  drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1);
  viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(viz_topic, 1);

  timer_ = nh_.createTimer(ros::Duration(0.1), &DriveController::timerCallback, this);

  pure_pursuit_ = std::make_unique<PurePursuit>(look_ahead_dist, gain);
}

void DriveController::timerCallback(const ros::TimerEvent& timer_event)
{
  ackermann_msgs::AckermannDriveStamped drive_msg;

  try
  {
    drive_msg.drive.steering_angle = pure_pursuit_->calculateSteeringAngle(odom_msg_, path_);
    ROS_INFO_STREAM("[Drive Controller] Steering Angle: " << drive_msg.drive.steering_angle * 180.0 / M_PI
                                                          << " degrees");

    geometry_msgs::PointStamped look_ahead_point;
    geometry_msgs::PointStamped arc_center;
    double look_ahead_point_dist;
    double arc_radius;

    pure_pursuit_->getIntermediateResults(look_ahead_point, look_ahead_point_dist, arc_center, arc_radius);
    publishVisualization(look_ahead_point_dist, look_ahead_point, arc_center, arc_radius);
  }
  catch (const std::runtime_error& ex)
  {
    ROS_ERROR("[Drive Controller] %s", ex.what());
    return;
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

  drive_msg.drive.speed = 2.0;

  drive_pub_.publish(drive_msg);
}

void DriveController::odomCallback(const nav_msgs::Odometry odom_msg)
{
  odom_msg_ = odom_msg;
}

void DriveController::pathCallback(const nav_msgs::Path path_msg)
{
  path_ = path_msg;
}

void DriveController::publishVisualization(const double search_radius,
                                           const geometry_msgs::PointStamped look_ahead_point,
                                           const geometry_msgs::PointStamped arc_center, const double arc_radius)
{
  visualization_msgs::MarkerArray markers;

  visualization_msgs::Marker search_radius_marker;
  search_radius_marker.header.frame_id = odom_msg_.header.frame_id;
  search_radius_marker.action = visualization_msgs::Marker::ADD;
  search_radius_marker.type = visualization_msgs::Marker::CYLINDER;
  search_radius_marker.id = 0;
  search_radius_marker.lifetime = ros::Duration(0.1);

  search_radius_marker.pose = odom_msg_.pose.pose;
  search_radius_marker.scale.x = 2 * search_radius;
  search_radius_marker.scale.y = 2 * search_radius;
  search_radius_marker.scale.z = 0.001;

  search_radius_marker.color.r = 0;
  search_radius_marker.color.g = 1;
  search_radius_marker.color.b = 0;
  search_radius_marker.color.a = 0.3;

  markers.markers.push_back(search_radius_marker);

  visualization_msgs::Marker look_ahead_point_marker;
  look_ahead_point_marker.header.frame_id = look_ahead_point.header.frame_id;
  look_ahead_point_marker.action = visualization_msgs::Marker::ADD;
  look_ahead_point_marker.type = visualization_msgs::Marker::SPHERE;
  look_ahead_point_marker.id = 1;
  look_ahead_point_marker.lifetime = ros::Duration(0.1);

  look_ahead_point_marker.pose.position = look_ahead_point.point;
  look_ahead_point_marker.scale.x = 0.1;
  look_ahead_point_marker.scale.y = 0.1;
  look_ahead_point_marker.scale.z = 0.1;

  look_ahead_point_marker.color.r = 0;
  look_ahead_point_marker.color.g = 0;
  look_ahead_point_marker.color.b = 1;
  look_ahead_point_marker.color.a = 1;
  markers.markers.push_back(look_ahead_point_marker);

  visualization_msgs::Marker arc_center_marker;
  arc_center_marker.header.frame_id = arc_center.header.frame_id;
  arc_center_marker.action = visualization_msgs::Marker::ADD;
  arc_center_marker.type = visualization_msgs::Marker::SPHERE;
  arc_center_marker.id = 2;
  arc_center_marker.lifetime = ros::Duration(0.1);

  arc_center_marker.pose.position = arc_center.point;

  arc_center_marker.scale.x = 0.1;
  arc_center_marker.scale.y = 0.1;
  arc_center_marker.scale.z = 0.1;

  arc_center_marker.color.r = 1;
  arc_center_marker.color.g = 1;
  arc_center_marker.color.b = 1;
  arc_center_marker.color.a = 1;
  markers.markers.push_back(arc_center_marker);

  visualization_msgs::Marker arc_marker;
  arc_marker.header.frame_id = arc_center.header.frame_id;
  arc_marker.action = visualization_msgs::Marker::ADD;
  arc_marker.type = visualization_msgs::Marker::LINE_STRIP;
  arc_marker.id = 3;
  arc_marker.lifetime = ros::Duration(0.1);

  double steps = 50;
  double theta_start = -M_PI_2 * 1.1;
  double theta_end = M_PI_2 * 1.1;
  double theta_step = (theta_end - theta_start) / steps;

  for (int i = 0; i < steps; i++)
  {
    double theta = theta_start + theta_step * i;
    geometry_msgs::Point point;
    point.x = arc_center.point.x + arc_radius * cos(theta);
    point.y = arc_center.point.y + arc_radius * sin(theta);
    arc_marker.points.push_back(point);
  }

  arc_marker.scale.x = 0.02;

  arc_marker.color.r = 1;
  arc_marker.color.g = 1;
  arc_marker.color.b = 1;
  arc_marker.color.a = 1;
  markers.markers.push_back(arc_marker);

  viz_pub_.publish(markers);
}
}  // namespace control
}  // namespace f1tenth_racecar

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "drive_controller");
  f1tenth_racecar::control::DriveController drive_controller;
  ros::spin();
  return 0;
}
