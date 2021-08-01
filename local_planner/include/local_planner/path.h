#ifndef LOCAL_PLANNER_PATH_H
#define LOCAL_PLANNER_PATH_H

#include <string>
#include <vector>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

class Path
{
protected:
  std::string frame_id_;

  std::vector<double> distance_;
  std::vector<double> x_;
  std::vector<double> y_;
  std::vector<double> yaw_;
  std::vector<double> curvature_;

  size_t size_;

public:
  Path();

  Path(const std::string& frame_id, const std::vector<double>& distance, const std::vector<double>& x,
       const std::vector<double>& y, const std::vector<double>& yaw, const std::vector<double>& curvature);

  // explicit Path(const Path& path);

  std::string getFrameId() const;

  size_t size() const;

  double distance(size_t n) const;

  double x(size_t n) const;

  double y(size_t n) const;

  double yaw(size_t n) const;

  double curvature(size_t n) const;

  geometry_msgs::Point point(size_t n) const;

  geometry_msgs::Pose pose(size_t n) const;

  geometry_msgs::PoseStamped poseStamped(size_t n) const;

  virtual visualization_msgs::Marker generatePathMarker(const int marker_id, const std::string& ns, const double scale,
                                                        const double r, const double g, const double b,
                                                        const double a = 1.0) const;
};

#endif  // LOCAL_PLANNER_PATH_H
