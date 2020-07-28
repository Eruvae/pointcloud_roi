#include <ros/ros.h>
#include <pointcloud_roi_msgs/PointcloudWithRoi.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <eigen_conversions/eigen_msg.h>

pcl::visualization::PCLVisualizer *viewer = nullptr;
int vp1 = 0, vp2 = 1;
boost::mutex viewer_mtx;

void pcRoiCallback(const pointcloud_roi_msgs::PointcloudWithRoi &pc_roi)
{
  ROS_INFO_STREAM("Message received");

  pcl::PCLPointCloud2::Ptr pcl_cloud(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(pc_roi.cloud, *pcl_cloud);
  pcl::IndicesPtr inds(new std::vector<int>);
  *inds = pc_roi.roi_indices;
  pcl::ExtractIndices<pcl::PCLPointCloud2> extract;
  extract.setInputCloud(pcl_cloud);
  extract.setIndices(inds);


  pcl::PCLPointCloud2::Ptr roi_region_cloud(new pcl::PCLPointCloud2);
  pcl::PCLPointCloud2::Ptr no_roi_region_cloud(new pcl::PCLPointCloud2);
  extract.filter(*roi_region_cloud);
  extract.setNegative(true);
  extract.filter(*no_roi_region_cloud);

  pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2>::Ptr main_ch, no_roi_ch, roi_ch;
  bool has_rgb = false;
  for (size_t i = 0; i < pcl_cloud->fields.size(); ++i)
  {
    if (pcl_cloud->fields[i].name == "rgb" || pcl_cloud->fields[i].name == "rgba")
    {
      has_rgb = true;
      break;
    }
  }

  if (has_rgb)
    main_ch.reset(new pcl::visualization::PointCloudColorHandlerRGBField<pcl::PCLPointCloud2>(pcl_cloud));
  else
    main_ch.reset(new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PCLPointCloud2>(pcl_cloud, "z"));

  no_roi_ch.reset(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PCLPointCloud2>(no_roi_region_cloud, 255, 0, 0)); //This will display the point cloud in red (R,G,B)
  roi_ch.reset(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PCLPointCloud2>(roi_region_cloud, 0, 255, 0)); //This will display the point cloud in green (R,G,B)

  const geometry_msgs::Transform &t = pc_roi.transform;
  Eigen::Vector4f sensor_origin(t.translation.x, t.translation.y, t.translation.z, 1);
  Eigen::Quaternion<float> sensor_orientation(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z);

  viewer_mtx.lock();
  viewer->removeAllPointClouds();
  viewer->addPointCloud(no_roi_region_cloud, no_roi_ch, sensor_origin, sensor_orientation, "no_roi_region", vp1);
  viewer->addPointCloud(roi_region_cloud, roi_ch, sensor_origin, sensor_orientation, "roi_region", vp1);
  viewer->addPointCloud(pcl_cloud, main_ch, sensor_origin, sensor_orientation, "ds_cloud", vp2);
  viewer_mtx.unlock();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "visualize_pwr");
  ros::NodeHandle nh;

  viewer = new pcl::visualization::PCLVisualizer("PC viewer");
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, vp1);
  viewer->setBackgroundColor (0, 0, 0, vp1);
  viewer->addText ("ROIs", 10, 10, "vp1cap", vp1);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, vp2);
  viewer->setBackgroundColor (0.1, 0.1, 0.1, vp2);
  viewer->addText ("Colored cloud", 10, 10, "vp2cap", vp2);

  ros::Subscriber pc_roi_sub = nh.subscribe("/detect_roi/results", 1, pcRoiCallback);

  for (ros::Rate rate(10); ros::ok(); rate.sleep()) // spin visualizer with 10 fps
  {
    ros::spinOnce();
    viewer_mtx.lock();
    viewer->spinOnce();
    viewer_mtx.unlock();
  }
}
