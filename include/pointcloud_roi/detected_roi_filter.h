#ifndef DETECTED_ROI_FILTER_H
#define DETECTED_ROI_FILTER_H

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/cache.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <yolact_ros_msgs/Detections.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <dynamic_reconfigure/server.h>
#include <pointcloud_roi_msgs/FilterDetectedRoiConfig.h>

namespace pointcloud_roi
{

class DetectedRoiFilter
{
public:
  DetectedRoiFilter(ros::NodeHandle &nhp);
  virtual ~DetectedRoiFilter();

private:
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, yolact_ros_msgs::Detections> DetsApproxSyncPolicy;
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, yolact_ros_msgs::Detections> DetsExactSyncPolicy;

  bool use_exact_sync;
  bool publish_colored_cloud;
  std::string target_frame;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> pc_sub;
  std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::PointCloud2>> transform_filter;
  std::unique_ptr<message_filters::Subscriber<yolact_ros_msgs::Detections>> det_sub;
  std::unique_ptr<message_filters::Synchronizer<DetsApproxSyncPolicy>> approx_sync;
  std::unique_ptr<message_filters::Synchronizer<DetsExactSyncPolicy>> exact_sync;
  //std::unique_ptr<message_filters::Cache<sensor_msgs::PointCloud2, yolact_ros_msgs::Detections> sync_cache;

  pointcloud_roi::FilterDetectedRoiConfig config;
  std::unique_ptr<dynamic_reconfigure::Server<pointcloud_roi::FilterDetectedRoiConfig>> dynrec_server;

  ros::Publisher pc_roi_pub;

  sensor_msgs::PointCloud2Ptr synced_pc;
  yolact_ros_msgs::DetectionsPtr synced_dets;
  std::mutex m;
  std::condition_variable cv;
  std::thread processing_thread;
  std::atomic_bool is_running;

  void processingThread();

  void detectionCallback(const sensor_msgs::PointCloud2Ptr &pc, const yolact_ros_msgs::DetectionsPtr &dets);

  template <typename PointT>
  void processDetections(const sensor_msgs::PointCloud2ConstPtr &pc, const yolact_ros_msgs::DetectionsConstPtr &dets);

  void reconfigureCallback(pointcloud_roi::FilterDetectedRoiConfig &config, uint32_t level);
};

} // namespace pointcloud_roi

#endif // DETECTED_ROI_FILTER_H
