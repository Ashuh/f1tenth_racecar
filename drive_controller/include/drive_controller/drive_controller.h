#ifndef DRIVE_CONTROLLER_DRIVE_CONTROLLER_H
#define DRIVE_CONTROLLER_DRIVE_CONTROLLER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "drive_controller/pure_pursuit.h"
#include "f1tenth_msgs/Trajectory.h"

namespace f1tenth_racecar
{
namespace control
{
class DriveController
{
private:
  ros::NodeHandle nh_;

  ros::Timer timer_;

  ros::Subscriber trajectory_sub_;
  ros::Subscriber odom_sub_;

  ros::Publisher drive_pub_;
  ros::Publisher viz_pub_;

  nav_msgs::Odometry odom_msg_;
  f1tenth_msgs::Trajectory trajectrory_;

  std::unique_ptr<PurePursuit> pure_pursuit_;

  void timerCallback(const ros::TimerEvent& timer_event);

  void odomCallback(const nav_msgs::Odometry odom_msg);

  void trajectoryCallback(const f1tenth_msgs::Trajectory traj_msg);

  visualization_msgs::Marker buildLookAheadDistMarker(const double look_ahead_dist);

  void publishVisualization(const double search_radius, const geometry_msgs::PointStamped look_ahead_point,
                            const geometry_msgs::PointStamped arc_center, const double arc_radius);

public:
  DriveController();
};
}  // namespace control
}  // namespace f1tenth_racecar

#endif  // DRIVE_CONTROLLER_DRIVE_CONTROLLER_H
