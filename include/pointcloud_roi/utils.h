#pragma once

#include <utility>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>

namespace pointcloud_roi
{

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> separateCloudByIndices(const typename pcl::PointCloud<PointT>::ConstPtr &input_cloud, const pcl::IndicesConstPtr &indices)
{
  typename pcl::PointCloud<PointT>::Ptr inlier_cloud(new pcl::PointCloud<PointT>), outlier_cloud(new pcl::PointCloud<PointT>);
  inlier_cloud->header = input_cloud->header;
  outlier_cloud->header = input_cloud->header;
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(input_cloud);
  extract.setIndices(indices);
  extract.setNegative(false);
  extract.filter(*inlier_cloud);
  extract.setNegative(true);
  extract.filter(*outlier_cloud);
  return std::make_pair(inlier_cloud, outlier_cloud);
}

}
