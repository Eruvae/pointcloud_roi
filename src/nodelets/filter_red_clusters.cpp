#include "pointcloud_roi/filter_red_clusters.h"
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

namespace pointcloud_roi
{

void FilterRedClustersNodelet::onInit()
{
  target_frame = getNodeHandle().param<std::string>("map_frame", "map");
  tf_buffer.reset(new tf2_ros::Buffer(ros::Duration(tf2::BufferCore::DEFAULT_CACHE_TIME)));
  tf_listener.reset(new tf2_ros::TransformListener(*tf_buffer, getPrivateNodeHandle()));
  pc_sub.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(getPrivateNodeHandle(), "input", 1));
  transform_filter.reset(new tf2_ros::MessageFilter<sensor_msgs::PointCloud2>(*pc_sub, *tf_buffer, target_frame, 1000, getPrivateNodeHandle()));
  transform_filter->registerCallback(&FilterRedClustersNodelet::pointcloudCallback, this);
  pc_roi_pub = getPrivateNodeHandle().advertise<pointcloud_roi_msgs::PointcloudWithRoi>("results", 1);
}

pcl::IndicesConstPtr FilterRedClustersNodelet::filterRed(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output)
{
  if (output == nullptr)
    output.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

  //inRange((30, 25, 15), (80, 255,255))
  //pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond(new pcl::ConditionAnd<pcl::PointXYZRGB>);
  //color_cond->addComparison(pcl::PackedHSIComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<pcl::PointXYZRGB> ("h", pcl::ComparisonOps::GE, 0)));
  //color_cond->addComparison(pcl::PackedHSIComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<pcl::PointXYZRGB> ("h", pcl::ComparisonOps::LE, 35)));
  //color_cond->addComparison(pcl::PackedHSIComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<pcl::PointXYZRGB> ("s", pcl::ComparisonOps::GE, 50)));
  //color_cond->addComparison(pcl::PackedHSIComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<pcl::PointXYZRGB> ("i", pcl::ComparisonOps::GE, 70)));
  pcl::ConditionOr<pcl::PointXYZRGB>::Ptr color_cond(new pcl::ConditionOr<pcl::PointXYZRGB>);
  color_cond->addComparison(pcl::PackedHSIComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<pcl::PointXYZRGB> ("h", pcl::ComparisonOps::LT, -20)));
  color_cond->addComparison(pcl::PackedHSIComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<pcl::PointXYZRGB> ("h", pcl::ComparisonOps::GT, 35)));
  color_cond->addComparison(pcl::PackedHSIComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<pcl::PointXYZRGB> ("s", pcl::ComparisonOps::LT, 30)));
  color_cond->addComparison(pcl::PackedHSIComparison<pcl::PointXYZRGB>::ConstPtr (new pcl::PackedHSIComparison<pcl::PointXYZRGB> ("i", pcl::ComparisonOps::LT, 30)));

  pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem(true);
  condrem.setInputCloud(input);
  condrem.setCondition(color_cond);
  condrem.filter(*output);
  return condrem.getRemovedIndices();
}

void FilterRedClustersNodelet::pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr &pc)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*pc, *pcl_cloud);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_ds(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  vg.setInputCloud (pcl_cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  //vg.setSaveLeafLayout(true);
  vg.filter (*pcl_cloud_ds);
  ROS_INFO_STREAM("PointCloud before: " << pcl_cloud->points.size()  << ", " << pcl_cloud_ds->points.size());

  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr not_red_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::IndicesConstPtr redIndices = filterRed(pcl_cloud_ds);
  pcl::IndicesPtr redIndicesRo(new std::vector<int>);

  ROS_INFO_STREAM("Red indices: " << redIndices->size());

  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> ro;
  ro.setInputCloud(pcl_cloud_ds);
  ro.setIndices(redIndices);
  ro.filter(*redIndicesRo);

  // Creating the KdTree object for the search method of the extraction
  /*pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (pcl_cloud_ds);

  std::vector<pcl::PointIndices> clusterIndices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (10);
  ec.setMaxClusterSize (2000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (pcl_cloud_ds);
  ec.setIndices(redIndices);
  ec.extract (clusterIndices);

  ROS_INFO_STREAM("Clusters: " << clusterIndices.size());
  for (size_t i = 0; i < clusterIndices.size(); i++)
  {
    ROS_INFO_STREAM("Cluster " << i << ": " << clusterIndices[i].indices.size());
  }*/

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
  /*for (const pcl::PointIndices &ind : clusterIndices)
  {
    res.roi_indices.insert(res.roi_indices.end(), ind.indices.begin(), ind.indices.end());
  }*/
  res.transform = pcFrameTf.transform;
  res.roi_indices.assign(redIndicesRo->begin(), redIndicesRo->end());
  pc_roi_pub.publish(res);
}

} // namespace pointcloud_roi

PLUGINLIB_EXPORT_CLASS(pointcloud_roi::FilterRedClustersNodelet, nodelet::Nodelet)
