#include <algorithm>
#include <limits>

#include "costmap_generator/costmap_value.h"
#include "local_planner/trajectory.h"
#include "local_planner/trajectory_evaluator.h"
#include "f1tenth_utils/math.h"
#include "f1tenth_utils/tf2_wrapper.h"

TrajectoryEvaluator::TrajectoryEvaluator(const std::shared_ptr<CollisionChecker> collision_checker_ptr,
                                         const double k_spatial, const double k_temporal,
                                         const double max_lateral_offset)
{
  if (collision_checker_ptr == nullptr)
  {
    throw std::invalid_argument("Collision checker cannot be nullptr");
  }

  collision_checker_ptr_ = collision_checker_ptr;
  k_spatial_ = k_spatial;
  max_lateral_offset_ = max_lateral_offset;
  // k_temporal_ = k_temporal;
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

  double obstacle_cost = 0;

  try
  {
    for (int i = 0; i < trajectory.size(); ++i)
    {
      double pose_cost = collision_checker_ptr_->checkCollision(trajectory.poseStamped(i));

      if (pose_cost == static_cast<int>(CostmapValue::OCCUPIED))
      {
        return std::numeric_limits<double>::max();
      }
      else
      {
        obstacle_cost = std::max(obstacle_cost, pose_cost);
      }
    }
  }
  catch (const std::exception& ex)
  {
    ROS_ERROR("[Local Planner] TTG Collision check failed due to %s. Assuming there is a collision.", ex.what());
    return std::numeric_limits<double>::max();
  }

  // Normalize cost to 0-1
  obstacle_cost /= static_cast<int>(CostmapValue::OCCUPIED);

  int last = trajectory.size() - 1;
  geometry_msgs::Point traj_end = trajectory.point(last);
  geometry_msgs::Point ref_traj_end = reference_trajectory_.point(last);
  double spatial_cost = calculateDistance(traj_end.x, traj_end.y, ref_traj_end.x, ref_traj_end.y) / max_lateral_offset_;
  // double temporal_cost = 0.0;

  // for (int i = 0; i < trajectory.size(); ++i)
  // {
  //   geometry_msgs::Pose relative_pose =
  //       TF2Wrapper::doTransform<geometry_msgs::Pose>(trajectory.pose(i), reference_trajectory_.pose(i));
  //   spatial_cost += std::abs(relative_pose.position.y);
  // temporal_cost += std::abs(trajectory.time(i) - reference_trajectory_.time(i));
  // }
  // return k_spatial_ * spatial_cost + k_temporal_ * temporal_cost;

  return k_spatial_ * spatial_cost + (1 - k_spatial_) * obstacle_cost;
}

void TrajectoryEvaluator::setReferenceTrajectory(const Trajectory& trajectory)
{
  reference_trajectory_ = trajectory;
}

void TrajectoryEvaluator::setWeights(const double k_spatial, const double k_temporal)
{
  k_spatial_ = k_spatial;
  // k_temporal_ = k_temporal;
}

void TrajectoryEvaluator::setMaxLateralOffset(const double offset)
{
  max_lateral_offset_ = offset;
}
