#ifndef LOCAL_PLANNER_TRAJECTORY_EVALUATOR_H
#define LOCAL_PLANNER_TRAJECTORY_EVALUATOR_H

#include <geometry_msgs/Pose.h>

#include "costmap_generator/collision_checker.h"
#include "local_planner/trajectory.h"

class TrajectoryEvaluator
{
private:
  double k_spatial_;
  // double k_temporal_;
  double max_lateral_offset_;

  Trajectory reference_trajectory_;
  std::shared_ptr<CollisionChecker> collision_checker_ptr_;

public:
  TrajectoryEvaluator(const std::shared_ptr<CollisionChecker> collision_checker_ptr, const double k_spatial,
                      const double k_temporal, const double max_lateral_offset);

  double evaluate(const Trajectory& trajectory) const;

  void setReferenceTrajectory(const Trajectory& trajectory);

  void setWeights(const double k_spatial, const double k_temporal);

  void setMaxLateralOffset(const double offset);
};

#endif  // LOCAL_PLANNER_TRAJECTORY_EVALUATOR_H
