#include <limits>

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

#include "drive_controller/pure_pursuit.h"
#include "f1tenth_msgs/Trajectory.h"
#include "f1tenth_utils/tf2_wrapper.h"

namespace f1tenth_racecar
{
namespace control
{
PurePursuit::PurePursuit(double look_ahead_dist, double gain) : look_ahead_dist_(look_ahead_dist), gain_(gain)
{
}

ackermann_msgs::AckermannDriveStamped PurePursuit::computeDrive(nav_msgs::Odometry odom,
                                                                const f1tenth_msgs::Trajectory trajectory)
{
  if (trajectory.waypoints.empty())
  {
    throw std::invalid_argument("Trajectory is empty");
  }

  int id = findLookAheadWaypointId(odom, trajectory);
  f1tenth_msgs::Waypoint look_ahead_wp = trajectory.waypoints.at(id);

  look_ahead_point_.header.frame_id = trajectory.header.frame_id;
  look_ahead_point_.point.x = look_ahead_wp.x;
  look_ahead_point_.point.y = look_ahead_wp.y;

  // Transform look ahead point to local frame
  geometry_msgs::PointStamped look_ahead_point_transformed =
      TF2Wrapper::doTransform(look_ahead_point_, odom.child_frame_id);

  ackermann_msgs::AckermannDriveStamped drive_msg;
  drive_msg.header.stamp = ros::Time::now();
  drive_msg.header.frame_id = odom.child_frame_id;
  drive_msg.drive.steering_angle = calculateSteeringAngle(look_ahead_point_transformed);
  drive_msg.drive.speed = look_ahead_wp.velocity;

  return drive_msg;
}

double PurePursuit::getDist(const geometry_msgs::Point point_1, const geometry_msgs::Point point_2)
{
  double d_x = point_1.x - point_2.x;
  double d_y = point_1.y - point_2.y;

  return sqrt(pow(d_x, 2) + pow(d_y, 2));
}

int PurePursuit::findLookAheadWaypointId(nav_msgs::Odometry odom, const f1tenth_msgs::Trajectory trajectory)
{
  look_ahead_point_dist_ = -std::numeric_limits<double>::max();

  int id = 0;

  for (int i = 0; i < trajectory.waypoints.size(); ++i)
  {
    f1tenth_msgs::Waypoint waypoint = trajectory.waypoints.at(i);
    geometry_msgs::Point point;
    point.x = waypoint.x;
    point.y = waypoint.y;
    double dist = getDist(point, odom.pose.pose.position);

    bool isWaypointAhead = TF2Wrapper::doTransform(point, odom.child_frame_id, waypoint.header.frame_id).x > 0;

    if (dist < look_ahead_dist_ && dist > look_ahead_point_dist_ && isWaypointAhead)
    {
      look_ahead_point_dist_ = dist;
      id = i;
    }
  }

  if (look_ahead_point_dist_ < 0)
  {
    throw std::runtime_error("No valid look ahead point found in path");
  }

  return id;
}

double PurePursuit::calculateSteeringAngle(const geometry_msgs::PointStamped look_ahead_point)

{
  arc_radius_ = pow(look_ahead_point_dist_, 2) / (2 * abs(look_ahead_point.point.y));
  double d = arc_radius_ - abs(look_ahead_point.point.y);
  arc_center_.header.frame_id = look_ahead_point.header.frame_id;
  arc_center_.point.x = 0.0;
  arc_center_.point.y = (look_ahead_point.point.y > 0) ? look_ahead_point.point.y + d : look_ahead_point.point.y - d;

  return ((look_ahead_point.point.y > 0) ? 1 : -1) * gain_ / arc_radius_;
}

void PurePursuit::getIntermediateResults(geometry_msgs::PointStamped& look_ahead_point, double& look_ahead_point_dist,
                                         geometry_msgs::PointStamped& arc_center, double& arc_radius)
{
  look_ahead_point = look_ahead_point_;
  look_ahead_point_dist = look_ahead_point_dist_;
  arc_center = arc_center_;
  arc_radius = arc_radius_;
}

void PurePursuit::setGain(const double gain)
{
  gain_ = gain;
}

void PurePursuit::setLookAheadDistance(const double distance)
{
  look_ahead_dist_ = distance;
}
}  // namespace control
}  // namespace f1tenth_racecar
