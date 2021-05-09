#include <vector>

#include <cv_bridge/rgb_colors.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/Polygon.h>
#include <sensor_msgs/PointCloud2.h>

#include "lidar_obstacle_detector/pcl_euclidean_clusterer.h"

namespace f1tenth_racecar
{
namespace perception
{
PCLEuclideanClusterer::PCLEuclideanClusterer()
{
}

void PCLEuclideanClusterer::cluster(sensor_msgs::PointCloud2 in_cloud_msg, sensor_msgs::PointCloud2& out_cloud_msg,
                                    std::vector<geometry_msgs::Polygon>& obstacles)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::moveFromROSMsg(in_cloud_msg, *pcl_input);

  std::vector<pcl::PointIndices> cluster_indices = generateClusterIndices(pcl_input);
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pcl_clusters = extractClusters(pcl_input, cluster_indices);
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pcl_clusters_rgb = colorClusters(pcl_clusters);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_clusters_rgb_merged = mergeClusters(pcl_clusters_rgb);

  sensor_msgs::PointCloud2 clusters_msg;
  pcl::toROSMsg(*pcl_clusters_rgb_merged, clusters_msg);

  obstacles = boundClusters(pcl_clusters);
  out_cloud_msg = clusters_msg;
}

std::vector<pcl::PointIndices>
PCLEuclideanClusterer::generateClusterIndices(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input)
{
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(pcl_input);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.20);
  ec.setMinClusterSize(5);
  ec.setMaxClusterSize(800);
  ec.setSearchMethod(tree);
  ec.setInputCloud(pcl_input);
  ec.extract(cluster_indices);

  return cluster_indices;
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> PCLEuclideanClusterer::extractClusters(
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input, std::vector<pcl::PointIndices> cluster_indices)
{
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pcl_clusters;

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
    {
      pcl_cluster->push_back((*pcl_input)[*pit]);  //*
    }

    pcl_cluster->width = pcl_cluster->size();
    pcl_cluster->height = 1;
    pcl_cluster->is_dense = true;

    pcl_clusters.push_back(pcl_cluster);
  }
  return pcl_clusters;
}

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>
PCLEuclideanClusterer::colorClusters(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pcl_clusters)
{
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pcl_clusters_rgb;

  for (int i = 0; i < pcl_clusters.size(); i++)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cluster_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*pcl_clusters.at(i), *pcl_cluster_rgb);

    cv::Vec3d cv_rgb = cv_bridge::rgb_colors::getRGBColor(i % 146);
    uint8_t r = cv_rgb[0] * 255, g = cv_rgb[1] * 255, b = cv_rgb[2] * 255;
    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);

    for (pcl::PointCloud<pcl::PointXYZRGB>::iterator it = pcl_cluster_rgb->begin(); it != pcl_cluster_rgb->end(); ++it)
    {
      it->rgb = *reinterpret_cast<float*>(&rgb);
    }

    pcl_clusters_rgb.push_back(pcl_cluster_rgb);
  }

  return pcl_clusters_rgb;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
PCLEuclideanClusterer::mergeClusters(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pcl_clusters_rgb)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_clusters_rgb_merged(new pcl::PointCloud<pcl::PointXYZRGB>);

  for (auto pcl_cluster : pcl_clusters_rgb)
  {
    *pcl_clusters_rgb_merged += *pcl_cluster;
  }

  return pcl_clusters_rgb_merged;
}

std::vector<geometry_msgs::Polygon>
PCLEuclideanClusterer::boundClusters(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pcl_clusters)
{
  std::vector<geometry_msgs::Polygon> obstacles;

  for (auto pcl_cluster : pcl_clusters)
  {
    std::vector<cv::Point2f> cluster_points;
    for (pcl::PointCloud<pcl::PointXYZ>::iterator it = pcl_cluster->begin(); it != pcl_cluster->end(); ++it)
    {
      cv::Point2f point(it->x, it->y);
      cluster_points.push_back(point);
    }
    cv::RotatedRect bounding_rect = cv::minAreaRect(cluster_points);

    cv::Point2f pts[4];
    bounding_rect.points(pts);
    geometry_msgs::Polygon obstacle;

    for (int i = 0; i < 4; i++)
    {
      geometry_msgs::Point32 point;
      point.x = pts[i].x;
      point.y = pts[i].y;
      obstacle.points.push_back(point);
    }

    obstacles.push_back(obstacle);
  }

  return obstacles;
}
}  // namespace perception
}  // namespace f1tenth_racecar
