#ifndef LIDAR_OBSTACLE_DETECTOR_PCL_EUCLIDEAN_CLUSTERER_H
#define LIDAR_OBSTACLE_DETECTOR_PCL_EUCLIDEAN_CLUSTERER_H

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <sensor_msgs/PointCloud2.h>

namespace f1tenth_racecar
{
namespace perception
{
class PCLEuclideanClusterer
{
private:
  std::vector<pcl::PointIndices> generateClusterIndices(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input);
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> extractClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input,
                                                                   std::vector<pcl::PointIndices> cluster_indices);
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>
  colorClusters(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pcl_clusters);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr
  mergeClusters(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pcl_clusters_rgb);

public:
  PCLEuclideanClusterer();

  sensor_msgs::PointCloud2 cluster(sensor_msgs::PointCloud2 in_cloud_msg);
};
}  // namespace perception
}  // namespace f1tenth_racecar

#endif  // LIDAR_OBSTACLE_DETECTOR_PCL_EUCLIDEAN_CLUSTERER_H
