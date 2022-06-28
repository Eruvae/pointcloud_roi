#include "pointcloud_roi/red_cluster_filter.h"
#include "pointcloud_roi/utils.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pointcloud_roi_msgs/PointcloudWithRoi.h>
#include <pcl/filters/radius_outlier_removal.h>

namespace pointcloud_roi
{

RedClusterFilter::RedClusterFilter(ros::NodeHandle &nhp)
{
  target_frame = nhp.param<std::string>("map_frame", "world");
  //bool transform_to_map_frame = nhp.param<bool>("publish_separate_clouds", false);
  //bool publish_combined_cloud = nhp.param<bool>("publish_combined_cloud", true);
  //bool publish_separate_clouds = nhp.param<bool>("publish_separate_clouds", false);

  tf_buffer.reset(new tf2_ros::Buffer(ros::Duration(tf2::BufferCore::DEFAULT_CACHE_TIME)));
  tf_listener.reset(new tf2_ros::TransformListener(*tf_buffer, nhp));
  pc_sub.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nhp, "input", 1));
  transform_filter.reset(new tf2_ros::MessageFilter<sensor_msgs::PointCloud2>(*pc_sub, *tf_buffer, target_frame, 1000, nhp));
  transform_filter->registerCallback(&RedClusterFilter::filter, this);
  pc_roi_pub = nhp.advertise<pointcloud_roi_msgs::PointcloudWithRoi>("results", 1);
  roi_only_pub = nhp.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("roi_cloud", 1);
  nonroi_only_pub = nhp.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("nonroi_cloud", 1);
  nhp.param("radius_filter/radius_search", radius_search, 0.02);
  nhp.param("radius_filter/min_neighbours", min_neighbours, 20);

}

pcl::IndicesConstPtr RedClusterFilter::filterRed(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output)
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

void RedClusterFilter::filter(const sensor_msgs::PointCloud2ConstPtr &pc)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*pc, *pcl_cloud);

  pcl::IndicesConstPtr redIndices = filterRed(pcl_cloud);

  geometry_msgs::TransformStamped pcFrameTf, pcFrameTfNow;
  try
  {
    pcFrameTf = tf_buffer->lookupTransform(target_frame, pc->header.frame_id, pc->header.stamp);
  }
  catch (const tf2::TransformException &e)
  {
    ROS_ERROR_STREAM("Couldn't find transform to map frame: " << e.what());
    return;
  }
  try
  {
    ros::Time now = ros::Time::now();
    pcFrameTfNow = tf_buffer->lookupTransform(target_frame, pc->header.frame_id, now, ros::Duration(3.0));
  }
  catch(const tf2::TransformException &e)
  {
    ROS_ERROR_STREAM("Couldn't find transform NOW to map frame: " << e.what());
    return;
  }
  
  ROS_INFO_STREAM("Transform for time " << pc->header.stamp << " successful");

  static Eigen::Isometry3d lastTfEigen;
  Eigen::Isometry3d tfEigen = tf2::transformToEigen(pcFrameTf);
  Eigen::Isometry3d tfEigenNow = tf2::transformToEigen(pcFrameTfNow);
  

  if (tfEigenNow.isApprox(tfEigen, 1e-2) && tfEigen.isApprox(lastTfEigen, 1e-2)) // Publish separate clouds only if not moved
  {
    const auto [inlier_cloud, outlier_cloud] = separateCloudByIndices<pcl::PointXYZRGB>(pcl_cloud, redIndices);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_pcl_nan_rem(new pcl::PointCloud<pcl::PointXYZRGB> );
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_pcl_filtered(new pcl::PointCloud<pcl::PointXYZRGB> );
    if(inlier_cloud->points.size() < 10)
    {
      ROS_WARN_THROTTLE(60, "Too few points, hence not processing further");
      return;
    }
    try
    {
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*inlier_cloud, *pc_pcl_nan_rem, indices);
      pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
      // build the filter
      outrem.setInputCloud(pc_pcl_nan_rem);
      outrem.setRadiusSearch(radius_search);
      outrem.setMinNeighborsInRadius(min_neighbours);
      outrem.setKeepOrganized(true);
      // apply filter
      outrem.filter (*pc_pcl_filtered);
    }
    catch(...)
    {
      ROS_ERROR_STREAM("Radius Outlier Removal Filter Error: \n");
      return;
    }
    roi_only_pub.publish(*pc_pcl_filtered);

    //nonroi_only_pub.publish(*outlier_cloud);
  }
  else
  {
    ROS_WARN_THROTTLE(30, "TFEigen Now and TF Eigen for PC stamp are different");
  }
  lastTfEigen = tfEigen;

  pcl::transformPointCloud(*pcl_cloud, *pcl_cloud, tfEigen.matrix());

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_ds(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  vg.setInputCloud (pcl_cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  //vg.setSaveLeafLayout(true);
  vg.filter (*pcl_cloud_ds);

  redIndices = filterRed(pcl_cloud_ds);
  pcl::IndicesPtr redIndicesRo(new std::vector<int>);

  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> ro;
  ro.setInputCloud(pcl_cloud_ds);
  ro.setIndices(redIndices);
  ro.filter(*redIndicesRo);

  

  pointcloud_roi_msgs::PointcloudWithRoi res;
  pcl::toROSMsg(*pcl_cloud, res.cloud);
  res.cloud.header.frame_id = target_frame;
  res.cloud.header.stamp = pc->header.stamp;
  res.transform = pcFrameTf.transform;
  res.roi_indices.assign(redIndicesRo->begin(), redIndicesRo->end());
  pc_roi_pub.publish(res);
}

} // namespace pointcloud_roi
