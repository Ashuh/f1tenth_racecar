#ifndef LOCAL_PLANNER_TRAJECTORY_EVALUATOR_H
#define LOCAL_PLANNER_TRAJECTORY_EVALUATOR_H

#include <geometry_msgs/Pose.h>

#include "local_planner/trajectory.h"

class TrajectoryEvaluator
{
private:
  double k_spatial_;
  double k_temporal_;

  Trajectory reference_trajectory_;

  double evaluateWaypoint(const geometry_msgs::Pose& ref_pose, const geometry_msgs::Pose pose) const;

public:
  TrajectoryEvaluator(const double k_spatial, const double k_temporal);

  double evaluate(const Trajectory& trajectory) const;

  void setReferenceTrajectory(const Trajectory& trajectory);

  void setWeights(const double k_spatial, const double k_temporal);
};

#endif  // LOCAL_PLANNER_TRAJECTORY_EVALUATOR_H
