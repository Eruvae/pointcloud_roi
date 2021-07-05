#include "pointcloud_roi/filter_nodelet.h"
#include "pointcloud_roi/detected_roi_filter.h"
#include "pointcloud_roi/red_cluster_filter.h"
#include <pluginlib/class_list_macros.h>

namespace pointcloud_roi
{
using FilterRedClustersNodelet = FilterNodelet<RedClusterFilter>;
using FilterDetectedRoiNodelet = FilterNodelet<DetectedRoiFilter>;
}

PLUGINLIB_EXPORT_CLASS(pointcloud_roi::FilterRedClustersNodelet, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(pointcloud_roi::FilterDetectedRoiNodelet, nodelet::Nodelet)
