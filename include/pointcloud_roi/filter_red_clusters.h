#ifndef FILTER_RED_CLUSTERS_H
#define FILTER_RED_CLUSTERS_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>

namespace pointcloud_roi
{

class FilterRedClustersNodelet : public nodelet::Nodelet
{
public:
  FilterRedClustersNodelet() : Nodelet() {}

private:
  std::string target_frame;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> pc_sub;
  std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::PointCloud2>> transform_filter;

  ros::Publisher pc_roi_pub;

  virtual void onInit();

  pcl::IndicesConstPtr filterRed(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output = nullptr);
  void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr &pc);

};

} // namespace pointcloud_roi

#endif // FILTER_RED_CLUSTERS_H
