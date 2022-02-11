#include <limits>

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

#include "drive_controller/pure_pursuit.h"
#include "f1tenth_msgs/Trajectory.h"
#include "f1tenth_utils/math.h"
#include "f1tenth_utils/tf2_wrapper.h"

PurePursuit::PurePursuit(const double look_ahead_dist, const double gain,
                         const std::shared_ptr<visualization_msgs::MarkerArray>& viz_ptr)
{
  look_ahead_dist_ = look_ahead_dist;
  gain_ = gain;
  viz_ptr_ = viz_ptr;
}

ackermann_msgs::AckermannDriveStamped PurePursuit::computeDrive(const geometry_msgs::Pose& cur_pose,
                                                                const f1tenth_msgs::Trajectory& trajectory,
                                                                const std::string& vehicle_frame)
{
  if (trajectory.waypoints.empty())
  {
    throw std::invalid_argument("Trajectory is empty");
  }

  f1tenth_msgs::Waypoint look_ahead_wp = trajectory.waypoints.at(findLookAheadWaypointId(cur_pose, trajectory));

  geometry_msgs::PointStamped look_ahead_point;
  look_ahead_point.header.frame_id = trajectory.header.frame_id;
  look_ahead_point.point.x = look_ahead_wp.x;
  look_ahead_point.point.y = look_ahead_wp.y;
  visualizeLookAheadPoint(look_ahead_point);

  double curvature = calculateCurvature(cur_pose, look_ahead_point.point);
  visualizeArc(curvature, vehicle_frame);

  ackermann_msgs::AckermannDriveStamped drive_msg;
  drive_msg.header.stamp = ros::Time::now();
  drive_msg.header.frame_id = vehicle_frame;
  drive_msg.drive.steering_angle = gain_ * curvature;
  drive_msg.drive.speed = look_ahead_wp.velocity;

  return drive_msg;
}

int PurePursuit::findLookAheadWaypointId(const geometry_msgs::Pose& cur_pose,
                                         const f1tenth_msgs::Trajectory& trajectory)
{
  double max_dist = -std::numeric_limits<double>::max();

  int id = 0;

  for (int i = 0; i < trajectory.waypoints.size(); ++i)
  {
    f1tenth_msgs::Waypoint waypoint = trajectory.waypoints.at(i);
    geometry_msgs::Point point;
    point.x = waypoint.x;
    point.y = waypoint.y;
    double dist = calculateDistance(point.x, point.y, cur_pose.position.x, cur_pose.position.y);

    bool isWaypointAhead = TF2Wrapper::doTransform(point, cur_pose).x > 0;

    if (dist < look_ahead_dist_ && dist > max_dist && isWaypointAhead)
    {
      max_dist = dist;
      id = i;
    }
  }

  if (max_dist < 0)
  {
    throw std::invalid_argument("Trajectory does not contain a valid look ahead point");
  }

  return id;
}

void PurePursuit::setGain(const double gain)
{
  gain_ = gain;
}

void PurePursuit::setLookAheadDistance(const double distance)
{
  look_ahead_dist_ = distance;
}

void PurePursuit::visualizeArc(const double curvature, const std::string& robot_frame_id) const
{
  if (viz_ptr_ == nullptr)
  {
    return;
  }

  double radius = 1 / std::abs(curvature);

  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = robot_frame_id;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.id = 0;
  marker.ns = "arc";
  marker.lifetime = ros::Duration(0.1);
  marker.scale.x = 0.02;
  marker.color.r = 1;
  marker.color.g = 1;
  marker.color.b = 1;
  marker.color.a = 1;

  static constexpr int steps = 50;
  static constexpr double theta_start = -M_PI_2 * 1.1;
  static constexpr double theta_end = M_PI_2 * 1.1;
  static constexpr double theta_step = (theta_end - theta_start) / steps;
  double center_y = 1 / curvature;  // y coordinate of the arc center in robot frame

  for (int i = 0; i < steps; i++)
  {
    double theta = theta_start + theta_step * i;
    geometry_msgs::Point point;
    point.x = radius * cos(theta);
    point.y = center_y + radius * sin(theta);
    marker.points.push_back(point);
  }

  viz_ptr_->markers.push_back(marker);
}

void PurePursuit::visualizeLookAheadPoint(const geometry_msgs::PointStamped& point) const
{
  if (viz_ptr_ == nullptr)
  {
    return;
  }

  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = point.header.frame_id;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.id = 0;
  marker.ns = "look_ahead_point";
  marker.lifetime = ros::Duration(0.1);
  marker.pose.position = point.point;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.r = 0;
  marker.color.g = 0;
  marker.color.b = 1;
  marker.color.a = 1;

  viz_ptr_->markers.push_back(marker);
}
