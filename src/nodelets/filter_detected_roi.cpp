#include "pointcloud_roi/filter_detected_roi.h"
#include <pluginlib/class_list_macros.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl_ros/transforms.h>
#include <pointcloud_roi_msgs/PointcloudWithRoi.h>
#include <yolact_ros_msgs/mask_utils.h>

namespace pointcloud_roi
{

void FilterDetectedRoiNodelet::onInit()
{
  use_exact_sync = getPrivateNodeHandle().param<bool>("use_exact_sync", false);
  target_frame = getNodeHandle().param<std::string>("map_frame", "map");
  tf_buffer.reset(new tf2_ros::Buffer(ros::Duration(tf2::BufferCore::DEFAULT_CACHE_TIME)));
  tf_listener.reset(new tf2_ros::TransformListener(*tf_buffer, getPrivateNodeHandle()));
  pc_sub.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(getPrivateNodeHandle(), "pc", 1));
  transform_filter.reset(new tf2_ros::MessageFilter<sensor_msgs::PointCloud2>(*pc_sub, *tf_buffer, target_frame, 1000, getPrivateNodeHandle()));
  det_sub.reset(new message_filters::Subscriber<yolact_ros_msgs::Detections>(getPrivateNodeHandle(), "dets", 1));
  if (use_exact_sync)
  {
    exact_sync.reset(new message_filters::Synchronizer<DetsExactSyncPolicy>(DetsExactSyncPolicy(100), *transform_filter, *det_sub));
    exact_sync->registerCallback(&FilterDetectedRoiNodelet::detectionCallback<pcl::PointXYZ>, this);
  }
  else
  {
    approx_sync.reset(new message_filters::Synchronizer<DetsApproxSyncPolicy>(DetsApproxSyncPolicy(100), *transform_filter, *det_sub));
    approx_sync->registerCallback(&FilterDetectedRoiNodelet::detectionCallback<pcl::PointXYZ>, this);
  }
  pc_roi_pub = getPrivateNodeHandle().advertise<pointcloud_roi_msgs::PointcloudWithRoi>("results", 1);
}

template <typename PointT>
void FilterDetectedRoiNodelet::detectionCallback(const sensor_msgs::PointCloud2ConstPtr &pc, const yolact_ros_msgs::DetectionsConstPtr &dets)
{
  typename pcl::PointCloud<PointT>::Ptr pcl_cloud(new pcl::PointCloud<PointT>);
  pcl::fromROSMsg(*pc, *pcl_cloud);

  /*ros::Time sor_start = ros::Time::now();
  typename pcl::PointCloud<PointT>::Ptr pcl_cloud_sor(new pcl::PointCloud<PointT>);
  typename pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud(pcl_cloud);
  sor.filter(*pcl_cloud_sor);
  ROS_INFO_STREAM("Time for SOR: " << ros::Time::now() - sor_start);*/

  typename pcl::PointCloud<PointT>::Ptr pcl_cloud_ds(new pcl::PointCloud<PointT>);
  typename pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud (pcl_cloud);
  vg.setMinimumPointsNumberPerVoxel(100);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.setSaveLeafLayout(true);
  vg.filter (*pcl_cloud_ds);
  ROS_INFO_STREAM("PointCloud before: " << pcl_cloud->points.size()  << ", " << pcl_cloud_ds->points.size());

  // Creating the KdTree object for the search method of the extraction
  typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud (pcl_cloud_ds);

  std::vector<pcl::PointIndicesPtr> allIndices;

  for (const yolact_ros_msgs::Detection &det : dets->detections)
  {
    ROS_INFO_STREAM("Detection is processed");
    std::set<int> inds_set;
    for (int y = det.box.y1; y < det.box.y2; y++)
    {
      for (int x = det.box.x1; x < det.box.x2; x++)
      {
        if (yolact_ros_msgs::test(det.mask, x - det.box.x1, y - det.box.y1))
        {
          const PointT &point = pcl_cloud->at(x, y);
          //int i = vg.getCentroidIndex(point);
          Eigen::Vector3i gc = vg.getGridCoordinates(point.x, point.y, point.z);
          int i = vg.getCentroidIndexAt(gc);
          if (i >= 0)
          {
            inds_set.insert(i);
          }
          //inds_set.insert((y - det.box.y1) * pcl_cloud->width + (x - det.box.x1));
        }
      }
    }
    pcl::PointIndices::Ptr inds(new pcl::PointIndices);
    inds->indices.assign(inds_set.begin(), inds_set.end());
    allIndices.push_back(inds);

    /*std::vector<pcl::PointIndices> cluster_indices;
    typename pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (30);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (pcl_cloud_ds);
    ec.setIndices(inds);
    ec.extract (cluster_indices);

    if (cluster_indices.empty())
    {
      ROS_INFO("Object not found");
      continue;
    }

    pcl::PointIndicesPtr clusterPtr(new pcl::PointIndices);
    *clusterPtr = cluster_indices[0];
    allIndices.push_back(clusterPtr);*/

    //ROS_INFO_STREAM("Is sorted: " << std::is_sorted(clusterPtr->indices.begin(), clusterPtr->indices.end()));
  }

  for (const pcl::PointIndicesPtr &iptr : allIndices)
  {
    ROS_INFO_STREAM("Region points: " << iptr->indices.size());
  }

  pcl::PointIndicesPtr roiRegion;
  if (allIndices.size() == 0)
  {
    ROS_INFO("No ROI found");
    return;
  }
  roiRegion = allIndices[0];
  for (size_t i = 1; i < allIndices.size(); i++)
  {
    pcl::PointIndicesPtr tmp(new pcl::PointIndices);
    tmp->indices.resize(roiRegion->indices.size() + allIndices[i]->indices.size());
    auto it = std::set_union(roiRegion->indices.begin(), roiRegion->indices.end(), allIndices[i]->indices.begin(), allIndices[i]->indices.end(), tmp->indices.begin());
    tmp->indices.resize(it - tmp->indices.begin());
    roiRegion = tmp;
  }

  geometry_msgs::TransformStamped pcFrameTf;
  try
  {
    pcFrameTf = tf_buffer->lookupTransform(target_frame, pc->header.frame_id, pc->header.stamp);
  }
  catch (const tf2::TransformException &e)
  {
    ROS_ERROR_STREAM("Couldn't find transform to map frame: " << e.what());
    return;
  }
  ROS_INFO_STREAM("Transform for time " << pc->header.stamp << " successful");
  auto tfEigen = tf2::transformToEigen(pcFrameTf).matrix();

  pcl::transformPointCloud(*pcl_cloud_ds, *pcl_cloud_ds, tfEigen);

  pointcloud_roi_msgs::PointcloudWithRoi res;
  pcl::toROSMsg(*pcl_cloud_ds, res.cloud);
  res.cloud.header.frame_id = target_frame;
  res.cloud.header.stamp = pc->header.stamp;
  res.transform = pcFrameTf.transform;
  res.roi_indices.assign(roiRegion->indices.begin(), roiRegion->indices.end());
  pc_roi_pub.publish(res);
}

} // namespace pointcloud_roi

PLUGINLIB_EXPORT_CLASS(pointcloud_roi::FilterDetectedRoiNodelet, nodelet::Nodelet)
