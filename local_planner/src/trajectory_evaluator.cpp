#include "local_planner/trajectory.h"
#include "local_planner/trajectory_evaluator.h"
#include "f1tenth_utils/tf2_wrapper.h"

TrajectoryEvaluator::TrajectoryEvaluator(const double k_spatial, const double k_temporal)
{
  k_spatial_ = k_spatial;
  k_temporal_ = k_temporal;
}

double TrajectoryEvaluator::evaluate(const Trajectory& trajectory) const
{
  if (reference_trajectory_.size() == 0)
  {
    throw std::runtime_error("Reference trajectory has not been set");
  }

  if (trajectory.getFrameId() != reference_trajectory_.getFrameId())
  {
    throw std::invalid_argument("Trajectory frame does not match reference trajectory frame");
  }

  double spatial_cost = 0.0;
  double temporal_cost = 0.0;

  for (int i = 0; i < trajectory.size(); ++i)
  {
    geometry_msgs::Pose relative_pose =
        TF2Wrapper::doTransform<geometry_msgs::Pose>(trajectory.pose(i), reference_trajectory_.pose(i));
    spatial_cost += std::abs(relative_pose.position.y);
    temporal_cost += std::abs(trajectory.time(i) - reference_trajectory_.time(i));
  }

  return k_spatial_ * spatial_cost + k_temporal_ * temporal_cost;
}
void TrajectoryEvaluator::setReferenceTrajectory(const Trajectory& trajectory)
{
  reference_trajectory_ = trajectory;
}

void TrajectoryEvaluator::setWeights(const double k_spatial, const double k_temporal)
{
  k_spatial_ = k_spatial;
  k_temporal_ = k_temporal;
}
