#include "f1tenth_ll2_global_planner/global_planner.h"

#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>
#include <vector>

GlobalPlanner::GlobalPlanner()
{
  std::string map_file;
  double ll2_origin_lat;
  double ll2_origin_lon;

  ros::NodeHandle private_nh("~");
  private_nh.getParam("map_file", map_file);
  private_nh.getParam("ll2_origin_lat", ll2_origin_lat);
  private_nh.getParam("ll2_origin_lon", ll2_origin_lon);
  private_nh.getParam("lanelet_frame", lanelet_frame_id_);
  private_nh.getParam("map_frame", map_frame_id_);

  ROS_INFO("map_file: %s", map_file.c_str());
  ROS_INFO("ll2_origin_lat: %f", ll2_origin_lat);
  ROS_INFO("ll2_origin_lon: %f", ll2_origin_lon);

  loadMap(map_file, ll2_origin_lat, ll2_origin_lon);
  visualizer_ = LaneletVisualizer(lanelet_frame_id_);

  map_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("lanelet2_map", 1, true);
  path_pub_ = nh_.advertise<nav_msgs::Path>("global_path", 1, true);

  goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &GlobalPlanner::goalCallback, this);
  odom_sub_ = nh_.subscribe("odom", 1, &GlobalPlanner::odomCallback, this);

  visualization_msgs::MarkerArray map_markers = visualizer_.generateMapMarker(map_);
  map_viz_pub_.publish(map_markers);
}

void GlobalPlanner::goalCallback(const geometry_msgs::PoseStamped& goal_pose)
{
  ROS_INFO("Goal received");

  try
  {
    geometry_msgs::Pose cur_earth_pose = TF2Wrapper::doTransform(cur_pose_, lanelet_frame_id_).pose;
    geometry_msgs::Pose goal_earth_pose = TF2Wrapper::doTransform(goal_pose, lanelet_frame_id_).pose;

    lanelet::BasicPoint2d cur_point(cur_earth_pose.position.x, cur_earth_pose.position.y);
    lanelet::BasicPoint2d goal_point(goal_earth_pose.position.x, goal_earth_pose.position.y);

    // Lanelet nearest to vehicle
    lanelet::Lanelet cur_lanelet = lanelet::geometry::findNearest(map_->laneletLayer, cur_point, 1).front().second;
    lanelet::Lanelet goal_lanelet = lanelet::geometry::findNearest(map_->laneletLayer, goal_point, 1).front().second;

    nav_msgs::Path path = plan(cur_lanelet, goal_lanelet);
    path_pub_.publish(path);
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }
}

void GlobalPlanner::odomCallback(const nav_msgs::Odometry& pose)
{
  cur_pose_.header = pose.header;
  cur_pose_.pose = pose.pose.pose;
}

void GlobalPlanner::loadMap(const std::string& file, double lat, double lon)
{
  lanelet::projection::UtmProjector projector(lanelet::Origin({ lat, lon }));
  try
  {
    map_ = lanelet::load(file, projector);
  }
  catch (const lanelet::FileNotFoundError& e)
  {
    ROS_ERROR("%s", e.what());
  }
  ROS_INFO("Lanelet2 map loaded");
}

nav_msgs::Path GlobalPlanner::plan(const lanelet::ConstLanelet& from, const lanelet::ConstLanelet& to)
{
  lanelet::traffic_rules::TrafficRulesPtr trafficRules{ lanelet::traffic_rules::TrafficRulesFactory::create(
      lanelet::Locations::Germany, lanelet::Participants::Vehicle) };

  lanelet::routing::RoutingGraphPtr graph = lanelet::routing::RoutingGraph::build(*map_, *trafficRules);
  lanelet::Optional<lanelet::routing::LaneletPath> shortestPath = graph->shortestPath(from, to);

  if (!shortestPath.is_initialized())
  {
    ROS_ERROR("No path found");
    return {};
  }

  std::vector<geometry_msgs::PoseStamped> ll2_path_poses;

  for (auto& section : shortestPath.get())
  {
    std::vector<geometry_msgs::PoseStamped> section_poses = lineStringToPoses(section.centerline());
    ll2_path_poses.insert(ll2_path_poses.end(), section_poses.begin(), section_poses.end());
  }

  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = map_frame_id_;

  for (int i = 0; i < ll2_path_poses.size() - 1; ++i)
  {
    geometry_msgs::PoseStamped pose_from = ll2_path_poses.at(i);
    geometry_msgs::PoseStamped pose_to = ll2_path_poses.at(i + 1);
    auto interpolated_poses = interpolatePoses(pose_from, pose_to, 0.1);

    path.poses.insert(path.poses.end(), interpolated_poses.begin(), interpolated_poses.end());
  }
  return path;
}

std::vector<geometry_msgs::PoseStamped> GlobalPlanner::interpolatePoses(const geometry_msgs::PoseStamped& from,
                                                                        const geometry_msgs::PoseStamped& to,
                                                                        const double step_size)
{
  double distance =
      calculateDistance(from.pose.position.x, from.pose.position.y, to.pose.position.x, to.pose.position.y);

  // Number of points to interpolate excluding the start and end point
  int num_via_points = std::ceil(distance / step_size) - 1;
  double dx = to.pose.position.x - from.pose.position.x;
  double dy = to.pose.position.y - from.pose.position.y;
  double angle = atan2(dy, dx);
  double step_x = step_size * cos(angle);
  double step_y = step_size * sin(angle);

  std::vector<geometry_msgs::PoseStamped> poses;
  poses.push_back(from);

  for (int i = 1; i <= num_via_points; ++i)
  {
    geometry_msgs::PoseStamped pose = from;
    pose.pose.position.x += i * step_x;
    pose.pose.position.y += i * step_y;
    poses.push_back(pose);
  }

  poses.push_back(to);

  return poses;
}

std::vector<geometry_msgs::PoseStamped>
GlobalPlanner::lineStringToPoses(const lanelet::ConstLineString3d& line_string) const
{
  std::vector<geometry_msgs::PoseStamped> poses;

  for (int i = 0; i < line_string.size(); i++)
  {
    auto start = line_string[i];
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = lanelet_frame_id_;
    pose.pose.position.x = start.x();
    pose.pose.position.y = start.y();

    if (i < line_string.size() - 1)
    {
      auto end = line_string[i + 1];
      double yaw = atan2(end.y() - start.y(), end.x() - start.x());
      pose.pose.orientation = TF2Wrapper::quatFromYaw(yaw);
    }
    else
    {
      pose.pose.orientation = poses.back().pose.orientation;
    }

    poses.push_back(pose);
  }

  for (auto& pose : poses)
  {
    pose = TF2Wrapper::doTransform(pose, map_frame_id_);
  }

  return poses;
}
