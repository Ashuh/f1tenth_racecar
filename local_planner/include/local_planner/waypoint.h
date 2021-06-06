#ifndef LOCAL_PLANNER_WAYPOINT_H
#define LOCAL_PLANNER_WAYPOINT_H

struct Waypoint
{
  double x_;
  double y_;
  double yaw_;
  double velocity_;
  double curvature_;
};

#endif  // LOCAL_PLANNER_WAYPOINT_H
